#pragma once

#include "PathSignalReservations.h"
#include "Types.hpp"
#include <optional>
#include <span>

namespace OpenLoco::Vehicles::PathSignals
{
    enum class PendingReservationResult
    {
        noPendingRoute,
        blocked,
        reserved,
    };

    /** Returns whether the experimental PBS implementation is enabled. */
    bool isEnabled();

    /** Converts one routed track piece into resources understood by PBS. */
    RouteStep makeRouteStep(const World::Pos3& pos, uint16_t trackAndDirection);

    /**
     * Atomically reserves route for train. If the tail still occupies an older
     * reservation, the new route is appended to it.
     */
    bool tryReserve(EntityId train, std::span<const RouteStep> route);

    /**
     * Retries a recently blocked route without running pathfinding again.
     * Pending routes expire after a few attempts so changed orders can replan.
     */
    PendingReservationResult retryPending(EntityId train, const World::Pos3& pos, uint16_t trackAndDirection);

    /** Returns whether a train currently owns any protected route. */
    bool hasReservation(EntityId train);

    /** Whether topology invalidation requires this train to reserve afresh. */
    bool needsRebuild(EntityId train);

    /** Finds the connection previously selected for a junction on the route. */
    std::optional<uint16_t> getReservedConnection(
        EntityId train,
        const World::Pos3& pos,
        std::span<const uint16_t> availableConnections);

    /** Releases one piece after the rear of the train has cleared it. */
    void releaseStep(EntityId train, const World::Pos3& pos, uint16_t trackAndDirection);

    /** Releases every resource owned by a train, for pickup or reversal. */
    void releaseAll(EntityId train);

    /** Replaces a crashed train's future route with its occupied track pieces. */
    void protectCrashFootprint(EntityId train, std::span<const RouteStep> occupiedSteps);

    /**
     * Invalidates derived routes after real rail or signal topology changes.
     * Trains already on the map reacquire before crossing their next track
     * boundary, so this is safe even when construction occurs mid-block.
     */
    void onRailTopologyChanged();

    /** Clears derived state when a world or routing table is reset. */
    void reset();
}
