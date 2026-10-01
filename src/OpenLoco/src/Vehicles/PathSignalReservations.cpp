/**
 * Storage and conflict detection for path-based signal reservations.
 */

#include "Vehicles/PathSignalReservations.h"
#include <algorithm>

namespace OpenLoco::Vehicles::PathSignals
{
    /** Packs a map position into a stable key for the resource-claim index. */
    static uint64_t resourceKey(const World::Pos3& pos)
    {
        return static_cast<uint64_t>(static_cast<uint16_t>(pos.x))
            | (static_cast<uint64_t>(static_cast<uint16_t>(pos.y)) << 16)
            | (static_cast<uint64_t>(static_cast<uint16_t>(pos.z)) << 32);
    }

    /** Returns whether a route step identifies the requested physical track piece. */
    static bool isSameTrackPiece(const RouteStep& step, const World::Pos3& pos, const uint16_t trackAndDirection)
    {
        return step.pos == pos
            && (step.trackAndDirection & kBasicTrackAndDirectionMask) == (trackAndDirection & kBasicTrackAndDirectionMask);
    }

    bool ReservationTable::canReserve(const ReservationOwner owner, const std::span<const RouteStep> route) const
    {
        if (route.empty())
        {
            return false;
        }

        for (const auto& step : route)
        {
            const auto reservation = _reservations.find(owner);
            const auto alreadyReserved = reservation != _reservations.end()
                && std::ranges::any_of(reservation->second.route, [&](const RouteStep& candidate) {
                       return isSameTrackPiece(candidate, step.pos, step.trackAndDirection);
                   });
            for (const auto& resource : step.resources)
            {
                const auto claims = _claimsByPosition.find(resourceKey(resource.pos));
                if (claims == _claimsByPosition.end())
                {
                    continue;
                }
                const auto conflicts = std::ranges::any_of(claims->second, [&](const ResourceClaim& claim) {
                    const auto overlaps = (claim.connectionMask & resource.connectionMask) != 0;
                    return overlaps && (claim.owner != owner || !alreadyReserved);
                });
                if (conflicts)
                {
                    return false;
                }
            }
        }
        return true;
    }

    bool ReservationTable::tryReserve(const ReservationOwner owner, const std::span<const RouteStep> route)
    {
        // Validate first. Failure must leave the train's existing path intact,
        // otherwise it could lose protection while still moving.
        if (!canReserve(owner, route))
        {
            return false;
        }

        const auto existing = _reservations.find(owner);
        if (existing == _reservations.end())
        {
            _reservations.emplace(owner, Reservation{ { route.begin(), route.end() }, false });
            for (const auto& step : route)
            {
                addClaims(owner, step);
            }
            return true;
        }

        for (const auto& step : route)
        {
            const auto alreadyReserved = std::ranges::any_of(existing->second.route, [&](const RouteStep& candidate) {
                return isSameTrackPiece(candidate, step.pos, step.trackAndDirection);
            });
            if (!alreadyReserved)
            {
                existing->second.route.push_back(step);
                addClaims(owner, step);
            }
        }
        return true;
    }

    void ReservationTable::replaceWithBlocker(const ReservationOwner owner, const std::span<const RouteStep> route)
    {
        // Physical blockers are facts about the map, not mutually exclusive
        // route requests. Two crashed trains may occupy the same resource, so
        // install every footprint even when their claims overlap.
        releaseAll(owner);
        if (route.empty())
        {
            return;
        }

        Reservation replacement{ {}, true };
        replacement.route.reserve(route.size());
        for (const auto& step : route)
        {
            const auto duplicate = std::ranges::any_of(replacement.route, [&](const RouteStep& candidate) {
                return isSameTrackPiece(candidate, step.pos, step.trackAndDirection);
            });
            if (duplicate)
            {
                continue;
            }

            replacement.route.push_back(step);
            addClaims(owner, step);
        }
        if (!replacement.route.empty())
        {
            _reservations.emplace(owner, std::move(replacement));
        }
    }

    void ReservationTable::addClaims(const ReservationOwner owner, const RouteStep& step)
    {
        for (const auto& resource : step.resources)
        {
            _claimsByPosition[resourceKey(resource.pos)].push_back(ResourceClaim{ owner, resource.connectionMask });
        }
    }

    void ReservationTable::releaseClaims(const ReservationOwner owner, const RouteStep& step)
    {
        for (const auto& resource : step.resources)
        {
            const auto claims = _claimsByPosition.find(resourceKey(resource.pos));
            if (claims == _claimsByPosition.end())
            {
                continue;
            }
            const auto claim = std::ranges::find_if(claims->second, [&](const ResourceClaim& candidate) {
                return candidate.owner == owner && candidate.connectionMask == resource.connectionMask;
            });
            if (claim != claims->second.end())
            {
                claims->second.erase(claim);
            }
            if (claims->second.empty())
            {
                _claimsByPosition.erase(claims);
            }
        }
    }

    void ReservationTable::releaseStep(const ReservationOwner owner, const World::Pos3& pos, const uint16_t trackAndDirection)
    {
        const auto reservation = _reservations.find(owner);
        if (reservation == _reservations.end())
        {
            return;
        }

        const auto step = std::ranges::find_if(reservation->second.route, [&](const RouteStep& candidate) {
            return isSameTrackPiece(candidate, pos, trackAndDirection);
        });
        if (step != reservation->second.route.end())
        {
            releaseClaims(owner, *step);
            reservation->second.route.erase(step);
        }
        if (reservation->second.route.empty())
        {
            _reservations.erase(reservation);
        }
    }

    void ReservationTable::releaseAll(const ReservationOwner owner)
    {
        const auto reservation = _reservations.find(owner);
        if (reservation == _reservations.end())
        {
            return;
        }
        for (const auto& step : reservation->second.route)
        {
            releaseClaims(owner, step);
        }
        _reservations.erase(reservation);
    }

    void ReservationTable::clearRoutes()
    {
        for (auto reservation = _reservations.begin(); reservation != _reservations.end();)
        {
            if (reservation->second.isBlocker)
            {
                ++reservation;
                continue;
            }
            for (const auto& step : reservation->second.route)
            {
                releaseClaims(reservation->first, step);
            }
            reservation = _reservations.erase(reservation);
        }
    }

    void ReservationTable::clear()
    {
        _reservations.clear();
        _claimsByPosition.clear();
    }

    std::optional<uint16_t> ReservationTable::getReservedConnection(
        const ReservationOwner owner,
        const World::Pos3& pos,
        const std::span<const uint16_t> availableConnections) const
    {
        const auto reservation = _reservations.find(owner);
        if (reservation == _reservations.end())
        {
            return std::nullopt;
        }

        for (const auto& step : reservation->second.route)
        {
            if (step.pos != pos)
            {
                continue;
            }
            for (const auto connection : availableConnections)
            {
                if ((connection & kBasicTrackAndDirectionMask) == (step.trackAndDirection & kBasicTrackAndDirectionMask))
                {
                    // Return the live connection. It contains bridge, mod and
                    // signal flags that are intentionally not stored as identity.
                    return connection;
                }
            }
        }
        return std::nullopt;
    }

    bool ReservationTable::hasReservation(const ReservationOwner owner) const
    {
        return _reservations.contains(owner);
    }

    size_t ReservationTable::reservationCount() const
    {
        return _reservations.size();
    }

    std::vector<ReservationOwner> ReservationTable::owners() const
    {
        std::vector<ReservationOwner> result;
        result.reserve(_reservations.size());
        for (const auto& reservation : _reservations)
        {
            result.push_back(reservation.first);
        }
        return result;
    }

    std::vector<ReservationOwner> ReservationTable::routeOwners() const
    {
        std::vector<ReservationOwner> result;
        result.reserve(_reservations.size());
        for (const auto& [owner, reservation] : _reservations)
        {
            if (!reservation.isBlocker)
            {
                result.push_back(owner);
            }
        }
        return result;
    }
}
