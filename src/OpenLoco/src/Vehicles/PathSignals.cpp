/**
 * Adapts OpenLoco vehicles and track geometry to path reservations.
 */

#include "Vehicles/PathSignals.h"
#include "Config.h"
#include "Entities/EntityManager.h"
#include "Map/Track/Track.h"
#include "Map/Track/TrackData.h"
#include "Vehicles/Vehicle.h"
#include <algorithm>

namespace OpenLoco::Vehicles::PathSignals
{
    static ReservationTable _reservations;
    static std::optional<bool> _lastEnabledState;
    static std::vector<ReservationOwner> _ownersNeedingRebuild;
    struct PendingReservation
    {
        ReservationOwner owner{};
        World::Pos3 pos{};
        uint16_t trackAndDirection{};
        uint8_t attempts{};
        std::vector<RouteStep> route{};
    };
    static std::vector<PendingReservation> _pendingReservations;

    /** Converts a train entity identifier to the reservation core's owner type. */
    static ReservationOwner toOwner(const EntityId train)
    {
        return enumValue(train);
    }

    bool isEnabled()
    {
        const auto enabled = Config::get().enablePathBasedSignals;
        if (_lastEnabledState.has_value() && *_lastEnabledState != enabled)
        {
            // Never revive paths created for the previously selected
            // signalling model after a runtime configuration change.
            _reservations.clearRoutes();
            _ownersNeedingRebuild.clear();
            _pendingReservations.clear();
        }
        _lastEnabledState = enabled;
        return enabled;
    }

    RouteStep makeRouteStep(const World::Pos3& pos, const uint16_t trackAndDirection)
    {
        RouteStep result{ pos, trackAndDirection, {} };

        TrackAndDirection::_TrackAndDirection tad{ 0, 0 };
        tad._data = trackAndDirection & World::Track::AdditionalTaDFlags::basicTaDMask;

        auto trackStart = pos;
        if (tad.isReversed())
        {
            const auto& trackSize = World::TrackData::getUnkTrack(tad._data);
            trackStart += trackSize.pos;
            if (trackSize.rotationEnd < 12)
            {
                trackStart -= World::Pos3{ World::kRotationOffset[trackSize.rotationEnd], 0 };
            }
        }

        for (const auto& piece : World::TrackData::getTrackPiece(tad.id()))
        {
            auto piecePos = trackStart;
            piecePos += World::Pos3{ Math::Vector::rotate(World::Pos2{ piece.x, piece.y }, tad.cardinalDirection()), piece.z };
            result.resources.push_back(TrackResource{ piecePos, piece.connectFlags[tad.cardinalDirection()] });
        }
        return result;
    }

    /** Checks live vehicle components in case derived reservations are incomplete. */
    static bool isPhysicallyOccupied(const EntityId train, const std::span<const RouteStep> route)
    {
        for (const auto& step : route)
        {
            for (const auto& resource : step.resources)
            {
                for (auto* entity : EntityManager::EntityTileList(World::Pos2{ resource.pos }))
                {
                    const auto* vehicle = entity->asBase<VehicleBase>();
                    if (vehicle == nullptr || vehicle->getHead() == train || vehicle->mode != TransportMode::rail)
                    {
                        continue;
                    }
                    if (vehicle->has38Flags(Flags38::unk_0 | Flags38::unk_2))
                    {
                        continue;
                    }

                    const auto occupied = makeRouteStep(vehicle->getTrackLoc(), vehicle->getTrackAndDirection().track._data);
                    const auto conflict = std::ranges::any_of(occupied.resources, [&](const TrackResource& other) {
                        return resource.pos == other.pos && (resource.connectionMask & other.connectionMask) != 0;
                    });
                    if (conflict)
                    {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    bool canReserve(const EntityId train, const std::span<const RouteStep> route)
    {
        if (!isEnabled() || train == EntityId::null || route.empty())
        {
            return false;
        }
        const auto owner = toOwner(train);
        return _reservations.canReserve(owner, route) && !isPhysicallyOccupied(train, route);
    }

    bool tryReserve(const EntityId train, const std::span<const RouteStep> route)
    {
        if (!isEnabled() || train == EntityId::null || route.empty())
        {
            return false;
        }
        if (!canReserve(train, route))
        {
            const auto owner = toOwner(train);
            const auto pending = std::ranges::find(_pendingReservations, owner, &PendingReservation::owner);
            const auto& first = route.front();
            const auto basicTrackAndDirection = static_cast<uint16_t>(first.trackAndDirection & kBasicTrackAndDirectionMask);
            const auto replacement = PendingReservation{
                owner,
                first.pos,
                basicTrackAndDirection,
                0,
                { route.begin(), route.end() },
            };
            if (pending == _pendingReservations.end())
            {
                _pendingReservations.push_back(replacement);
            }
            else
            {
                *pending = replacement;
            }
            return false;
        }
        const auto owner = toOwner(train);
        if (!_reservations.tryReserve(owner, route))
        {
            return false;
        }
        std::erase_if(_pendingReservations, [owner](const PendingReservation& pending) { return pending.owner == owner; });
        std::erase(_ownersNeedingRebuild, owner);
        return true;
    }

    PendingReservationResult retryPending(const EntityId train, const World::Pos3& pos, const uint16_t trackAndDirection)
    {
        if (!isEnabled() || train == EntityId::null)
        {
            return PendingReservationResult::noPendingRoute;
        }

        constexpr uint8_t kAttemptsBeforeReplan = 8;
        const auto owner = toOwner(train);
        const auto pending = std::ranges::find(_pendingReservations, owner, &PendingReservation::owner);
        if (pending == _pendingReservations.end())
        {
            return PendingReservationResult::noPendingRoute;
        }
        const auto isSameEntryPoint = pending->pos == pos
            && pending->trackAndDirection == (trackAndDirection & kBasicTrackAndDirectionMask);
        const auto shouldReplan = ++pending->attempts >= kAttemptsBeforeReplan;
        if (!isSameEntryPoint || shouldReplan)
        {
            _pendingReservations.erase(pending);
            return PendingReservationResult::noPendingRoute;
        }
        const auto hasReservationConflict = !_reservations.canReserve(owner, pending->route);
        if (hasReservationConflict || isPhysicallyOccupied(train, pending->route) || !_reservations.tryReserve(owner, pending->route))
        {
            return PendingReservationResult::blocked;
        }

        _pendingReservations.erase(pending);
        std::erase(_ownersNeedingRebuild, owner);
        return PendingReservationResult::reserved;
    }

    bool hasReservation(const EntityId train)
    {
        return train != EntityId::null && _reservations.hasReservation(toOwner(train));
    }

    bool needsRebuild(const EntityId train)
    {
        return train != EntityId::null
            && std::ranges::find(_ownersNeedingRebuild, toOwner(train)) != _ownersNeedingRebuild.end();
    }

    std::optional<uint16_t> getReservedConnection(
        const EntityId train,
        const World::Pos3& pos,
        const std::span<const uint16_t> availableConnections)
    {
        if (!isEnabled())
        {
            return std::nullopt;
        }
        return _reservations.getReservedConnection(toOwner(train), pos, availableConnections);
    }

    void releaseStep(const EntityId train, const World::Pos3& pos, const uint16_t trackAndDirection)
    {
        if (train != EntityId::null)
        {
            _reservations.releaseStep(toOwner(train), pos, trackAndDirection);
        }
    }

    void releaseAll(const EntityId train)
    {
        if (train != EntityId::null)
        {
            const auto owner = toOwner(train);
            _reservations.releaseAll(owner);
            std::erase(_ownersNeedingRebuild, owner);
            std::erase_if(_pendingReservations, [owner](const PendingReservation& pending) { return pending.owner == owner; });
        }
    }

    void protectCrashFootprint(const EntityId train, const std::span<const RouteStep> occupiedSteps)
    {
        if (train == EntityId::null)
        {
            return;
        }

        const auto owner = toOwner(train);
        _reservations.replaceWithBlocker(owner, occupiedSteps);
        std::erase(_ownersNeedingRebuild, owner);
        std::erase_if(_pendingReservations, [owner](const PendingReservation& pending) { return pending.owner == owner; });
    }

    void onRailTopologyChanged()
    {
        // A signal is a route boundary. Adding or removing one can change the
        // correct extent of paths far beyond the edited tile, so selective
        // invalidation of only that tile would be unsafe.
        for (const auto owner : _reservations.routeOwners())
        {
            if (std::ranges::find(_ownersNeedingRebuild, owner) == _ownersNeedingRebuild.end())
            {
                _ownersNeedingRebuild.push_back(owner);
            }
        }
        _reservations.clearRoutes();
        _pendingReservations.clear();
    }

    void reset()
    {
        _reservations.clear();
        _ownersNeedingRebuild.clear();
        _pendingReservations.clear();
        _lastEnabledState.reset();
    }
}
