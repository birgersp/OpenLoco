#pragma once

#include <OpenLoco/Engine/World.hpp>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <sfl/static_vector.hpp>
#include <span>
#include <unordered_map>
#include <vector>

namespace OpenLoco::Vehicles::PathSignals
{
    using ReservationOwner = uint16_t;
    // Kept local to the generic reservation layer so it does not have to pull
    // in the map graph and its container dependencies.
    constexpr uint16_t kBasicTrackAndDirectionMask = 0x01FF;

    /**
     * A small part of the map that a train needs exclusive use of.
     *
     * Track pieces that cross on the same tile share one or more bits in
     * connectionMask. Keeping the mask here lets the reservation code detect a
     * crossing without knowing anything about Locomotion's track-piece table.
     */
    struct TrackResource
    {
        World::Pos3 pos{};
        uint8_t connectionMask{};
    };

    // The largest Locomotion rail piece occupies five map tiles. Keeping
    // these resources inline avoids a heap allocation for every route step.
    using TrackResources = sfl::static_vector<TrackResource, 5>;

    /** One routed track piece and the map resources occupied by that piece. */
    struct RouteStep
    {
        World::Pos3 pos{};
        uint16_t trackAndDirection{};
        TrackResources resources{};
    };

    /**
     * Stores path reservations without depending on vehicles, signals, or the
     * global game state. This deliberately small interface isolates the policy
     * that a future extension API would need to replace or expose.
     */
    class ReservationTable
    {
    public:
        /** Checks whether route conflicts without modifying existing reservations. */
        bool canReserve(ReservationOwner owner, std::span<const RouteStep> route) const;

        /** Atomically adds route to an owner's reservation if it has no conflicts. */
        bool tryReserve(ReservationOwner owner, std::span<const RouteStep> route);

        /** Replaces an owner's planned route with its physical crash footprint. */
        void replaceWithBlocker(ReservationOwner owner, std::span<const RouteStep> route);

        /** Releases one routed piece after the owner's tail has left it. */
        void releaseStep(ReservationOwner owner, const World::Pos3& pos, uint16_t trackAndDirection);

        /** Releases every planned or physical claim belonging to an owner. */
        void releaseAll(ReservationOwner owner);

        /** Clears planned routes while preserving physical crash blockers. */
        void clearRoutes();

        /** Clears all reservations, including physical crash blockers. */
        void clear();

        std::optional<uint16_t> getReservedConnection(
            ReservationOwner owner,
            const World::Pos3& pos,
            std::span<const uint16_t> availableConnections) const;

        bool hasReservation(ReservationOwner owner) const;
        size_t reservationCount() const;
        std::vector<ReservationOwner> owners() const;

        /** Returns only owners of planned routes, excluding crash blockers. */
        std::vector<ReservationOwner> routeOwners() const;

    private:
        struct Reservation
        {
            std::vector<RouteStep> route{};
            bool isBlocker{};
        };

        struct ResourceClaim
        {
            ReservationOwner owner{};
            uint8_t connectionMask{};
        };

        void addClaims(ReservationOwner owner, const RouteStep& step);
        void releaseClaims(ReservationOwner owner, const RouteStep& step);

        std::unordered_map<ReservationOwner, Reservation> _reservations{};
        std::unordered_map<uint64_t, std::vector<ResourceClaim>> _claimsByPosition{};
    };
}
