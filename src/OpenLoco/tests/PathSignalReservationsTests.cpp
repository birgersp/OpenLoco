/**
 * Unit tests for path-reservation conflicts, release, and crash blockers.
 */

#include <OpenLoco/Vehicles/PathSignalReservations.h>
#include <array>
#include <gtest/gtest.h>

using namespace OpenLoco;
using namespace OpenLoco::Vehicles::PathSignals;

namespace
{
    /** Creates a one-tile route step for focused reservation-table tests. */
    RouteStep step(const World::Pos3 pos, const uint16_t tad, const uint8_t mask)
    {
        return RouteStep{ pos, tad, { TrackResource{ pos, mask } } };
    }
}

TEST(PathSignalReservationsTest, AllowsIndependentPathsThroughOneTile)
{
    ReservationTable reservations;
    const std::vector first{ step({ 32, 32, 16 }, 0, 0b0000'0011) };
    const std::vector second{ step({ 32, 32, 16 }, 8, 0b0000'1100) };

    EXPECT_TRUE(reservations.tryReserve(1, first));
    EXPECT_TRUE(reservations.tryReserve(2, second));
    EXPECT_EQ(reservations.reservationCount(), 2u);
}

TEST(PathSignalReservationsTest, RejectsCrossingPathsAtomically)
{
    ReservationTable reservations;
    const std::vector first{ step({ 32, 32, 16 }, 0, 0b0000'0011) };
    const std::vector crossing{ step({ 32, 32, 16 }, 8, 0b0000'0010) };

    ASSERT_TRUE(reservations.tryReserve(1, first));
    EXPECT_FALSE(reservations.canReserve(2, crossing));
    EXPECT_FALSE(reservations.tryReserve(2, crossing));
    EXPECT_TRUE(reservations.hasReservation(1));
    EXPECT_FALSE(reservations.hasReservation(2));
}

TEST(PathSignalReservationsTest, FailedExtensionKeepsPreviousReservation)
{
    ReservationTable reservations;
    const std::vector original{ step({ 0, 0, 0 }, 0, 0b1) };
    const std::vector otherTrain{ step({ 64, 0, 0 }, 0, 0b1) };
    const std::vector blockedReplacement{ step({ 64, 0, 0 }, 8, 0b1) };

    ASSERT_TRUE(reservations.tryReserve(1, original));
    ASSERT_TRUE(reservations.tryReserve(2, otherTrain));
    EXPECT_FALSE(reservations.tryReserve(1, blockedReplacement));

    const std::array<uint16_t, 1> connections{ 0 };
    EXPECT_EQ(reservations.getReservedConnection(1, { 0, 0, 0 }, connections), 0);
}

TEST(PathSignalReservationsTest, ReleasesTrackOnlyAfterExplicitTailRelease)
{
    ReservationTable reservations;
    const std::vector route{
        step({ 0, 0, 0 }, 0, 0b1),
        step({ 32, 0, 0 }, 0, 0b1),
    };
    ASSERT_TRUE(reservations.tryReserve(1, route));

    reservations.releaseStep(1, { 0, 0, 0 }, 0);
    EXPECT_TRUE(reservations.hasReservation(1));

    const std::vector newlyFree{ step({ 0, 0, 0 }, 0, 0b1) };
    EXPECT_TRUE(reservations.tryReserve(2, newlyFree));

    reservations.releaseStep(1, { 32, 0, 0 }, 0);
    EXPECT_FALSE(reservations.hasReservation(1));
}

TEST(PathSignalReservationsTest, ExtendsReservationWithoutReleasingTrackBehindTail)
{
    ReservationTable reservations;
    const std::vector firstBlock{ step({ 0, 0, 0 }, 0, 0b1) };
    const std::vector secondBlock{ step({ 32, 0, 0 }, 0, 0b1) };

    ASSERT_TRUE(reservations.tryReserve(1, firstBlock));
    ASSERT_TRUE(reservations.tryReserve(1, secondBlock));

    const std::vector conflictsBehindTail{ step({ 0, 0, 0 }, 8, 0b1) };
    const std::vector conflictsAhead{ step({ 32, 0, 0 }, 8, 0b1) };
    EXPECT_FALSE(reservations.tryReserve(2, conflictsBehindTail));
    EXPECT_FALSE(reservations.tryReserve(2, conflictsAhead));
}

TEST(PathSignalReservationsTest, RejectsNewPathAcrossOwnersRetainedTrack)
{
    ReservationTable reservations;
    constexpr World::Pos3 kCrossing{ 32, 32, 0 };
    const RouteStep retained{
        { 0, 0, 0 },
        0,
        { TrackResource{ kCrossing, 0b0000'0011 } },
    };
    const RouteStep crossesRetained{
        { 32, 0, 0 },
        8,
        { TrackResource{ kCrossing, 0b0000'0010 } },
    };

    ASSERT_TRUE(reservations.tryReserve(1, std::vector{ retained }));
    EXPECT_FALSE(reservations.canReserve(1, std::vector{ crossesRetained }));
    EXPECT_FALSE(reservations.tryReserve(1, std::vector{ crossesRetained }));

    // Retrying an identical step remains idempotent.
    EXPECT_TRUE(reservations.tryReserve(1, std::vector{ retained }));
    EXPECT_EQ(reservations.reservationCount(), 1u);
}

TEST(PathSignalReservationsTest, ReturnsLiveConnectionFlagsForReservedJunction)
{
    ReservationTable reservations;
    const std::vector route{ step({ 32, 64, 0 }, 16, 0b1) };
    ASSERT_TRUE(reservations.tryReserve(1, route));

    constexpr auto kExtraFlag = uint16_t{ 1U << 13 };
    const std::array<uint16_t, 2> connections{ 8, static_cast<uint16_t>(16 | kExtraFlag) };
    EXPECT_EQ(reservations.getReservedConnection(1, { 32, 64, 0 }, connections), 16 | kExtraFlag);
}

TEST(PathSignalReservationsTest, ReleasingTrainDoesNotAffectOtherOwners)
{
    ReservationTable reservations;
    ASSERT_TRUE(reservations.tryReserve(1, std::vector{ step({ 0, 0, 0 }, 0, 0b1) }));
    ASSERT_TRUE(reservations.tryReserve(2, std::vector{ step({ 64, 0, 0 }, 0, 0b1) }));

    reservations.releaseAll(1);

    EXPECT_FALSE(reservations.hasReservation(1));
    EXPECT_TRUE(reservations.hasReservation(2));
    EXPECT_TRUE(reservations.tryReserve(3, std::vector{ step({ 0, 0, 0 }, 8, 0b1) }));
}

TEST(PathSignalReservationsTest, ClearingTopologyReservationsAllowsFreshOwners)
{
    ReservationTable reservations;
    ASSERT_TRUE(reservations.tryReserve(7, std::vector{ step({ 0, 0, 0 }, 0, 0b1) }));
    EXPECT_EQ(reservations.owners(), std::vector<ReservationOwner>{ 7 });

    reservations.clear();

    EXPECT_EQ(reservations.reservationCount(), 0u);
    EXPECT_TRUE(reservations.tryReserve(7, std::vector{ step({ 0, 0, 0 }, 8, 0b1) }));
}

TEST(PathSignalReservationsTest, CrashFootprintReleasesFutureRoute)
{
    ReservationTable reservations;
    const auto occupied = step({ 0, 0, 0 }, 0, 0b1);
    const auto future = step({ 32, 0, 0 }, 0, 0b1);
    ASSERT_TRUE(reservations.tryReserve(1, std::vector{ occupied, future }));

    reservations.replaceWithBlocker(1, std::vector{ occupied });

    EXPECT_FALSE(reservations.tryReserve(2, std::vector{ step({ 0, 0, 0 }, 8, 0b1) }));
    EXPECT_TRUE(reservations.tryReserve(2, std::vector{ step({ 32, 0, 0 }, 8, 0b1) }));
}

TEST(PathSignalReservationsTest, OverlappingCrashFootprintsRemainBlockedUntilEveryWreckIsRemoved)
{
    ReservationTable reservations;
    const std::vector footprint{ step({ 0, 0, 0 }, 0, 0b1) };
    const std::vector conflictingRoute{ step({ 0, 0, 0 }, 8, 0b1) };
    reservations.replaceWithBlocker(1, footprint);
    reservations.replaceWithBlocker(2, footprint);

    reservations.releaseAll(1);
    EXPECT_FALSE(reservations.tryReserve(3, conflictingRoute));

    reservations.releaseAll(2);
    EXPECT_TRUE(reservations.tryReserve(3, conflictingRoute));
}

TEST(PathSignalReservationsTest, ClearingPlannedRoutesPreservesCrashBlockers)
{
    ReservationTable reservations;
    const std::vector blocker{ step({ 0, 0, 0 }, 0, 0b1) };
    const std::vector planned{ step({ 32, 0, 0 }, 0, 0b1) };
    reservations.replaceWithBlocker(1, blocker);
    ASSERT_TRUE(reservations.tryReserve(2, planned));

    reservations.clearRoutes();

    EXPECT_EQ(reservations.owners(), std::vector<ReservationOwner>{ 1 });
    EXPECT_FALSE(reservations.tryReserve(3, std::vector{ step({ 0, 0, 0 }, 8, 0b1) }));
    EXPECT_TRUE(reservations.tryReserve(3, std::vector{ step({ 32, 0, 0 }, 8, 0b1) }));
}

TEST(PathSignalReservationsTest, ReleasingOneStepPreservesAnotherClaimOnTheSameResource)
{
    ReservationTable reservations;
    constexpr World::Pos3 kSharedResource{ 64, 64, 0 };
    const RouteStep first{ { 0, 0, 0 }, 0, { TrackResource{ kSharedResource, 0b1 } } };
    const RouteStep second{ { 32, 0, 0 }, 0, { TrackResource{ kSharedResource, 0b1 } } };
    const std::vector conflicting{ RouteStep{ { 64, 64, 0 }, 8, { TrackResource{ kSharedResource, 0b1 } } } };
    ASSERT_TRUE(reservations.tryReserve(1, std::vector{ first, second }));

    reservations.releaseStep(1, first.pos, first.trackAndDirection);
    EXPECT_FALSE(reservations.tryReserve(2, conflicting));

    reservations.releaseStep(1, second.pos, second.trackAndDirection);
    EXPECT_TRUE(reservations.tryReserve(2, conflicting));
}

TEST(PathSignalReservationsTest, ClearingManyRoutesPreservesOnlyCrashBlockers)
{
    ReservationTable reservations;
    constexpr ReservationOwner kRouteCount = 256;
    for (ReservationOwner owner = 1; owner <= kRouteCount; ++owner)
    {
        const auto x = static_cast<int16_t>(owner * World::kTileSize);
        ASSERT_TRUE(reservations.tryReserve(owner, std::vector{ step({ x, 0, 0 }, 0, 0b1) }));
    }
    constexpr ReservationOwner kBlockerOwner = kRouteCount + 1;
    const std::vector blocker{ step({ 0, 64, 0 }, 0, 0b1) };
    reservations.replaceWithBlocker(kBlockerOwner, blocker);

    reservations.clearRoutes();

    EXPECT_EQ(reservations.reservationCount(), 1u);
    EXPECT_TRUE(reservations.hasReservation(kBlockerOwner));
    for (ReservationOwner owner = 1; owner <= kRouteCount; ++owner)
    {
        EXPECT_FALSE(reservations.hasReservation(owner));
    }
    EXPECT_FALSE(reservations.tryReserve(kBlockerOwner + 1, std::vector{ step({ 0, 64, 0 }, 8, 0b1) }));
    EXPECT_TRUE(reservations.tryReserve(kBlockerOwner + 1, std::vector{ step({ 0, 96, 0 }, 8, 0b1) }));
}
