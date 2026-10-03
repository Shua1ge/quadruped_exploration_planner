#include <gtest/gtest.h>
#include <plan_manage/replan_fsm_utils.h>

TEST(ForwardGaitTracking, SharesSpatialLookaheadAndBoundsStationaryTail)
{
  using scan_planner::forwardGaitLookaheadTime;
  const auto line = [](double t) { return Eigen::Vector3d(.165 * t, 0., 0.); };
  const double target = forwardGaitLookaheadTime(1., 10., line);
  EXPECT_GE(.165 * (target - 1.), .15);
  EXPECT_LT(.165 * (target - 1.), .16);
  EXPECT_DOUBLE_EQ(forwardGaitLookaheadTime(9.9, 10., line), 10.);
  const auto stopped = [](double) { return Eigen::Vector3d::Zero().eval(); };
  EXPECT_DOUBLE_EQ(forwardGaitLookaheadTime(1., 1000., stopped), 3.);
}

TEST(StructuredRepair, FailedRejoinCannotPurchaseAnotherEscape)
{
  using scan_planner::structuredRepairMayStartAnotherEscape;
  EXPECT_TRUE(structuredRepairMayStartAnotherEscape(false, 0));
  EXPECT_TRUE(structuredRepairMayStartAnotherEscape(false, 1));
  EXPECT_FALSE(structuredRepairMayStartAnotherEscape(true, 1));
  EXPECT_FALSE(structuredRepairMayStartAnotherEscape(true, 3));
}

TEST(ForwardGaitProfile, MapsMeasuredResponseWithoutChangingStopOrReverse)
{
  using scan_planner::calibratedForwardGaitCommand;
  EXPECT_DOUBLE_EQ(calibratedForwardGaitCommand(0.0, .30, .165, .30), 0.0);
  EXPECT_DOUBLE_EQ(calibratedForwardGaitCommand(-.10, .30, .165, .30), -.10);
  EXPECT_DOUBLE_EQ(calibratedForwardGaitCommand(.01, .30, .165, .30), .30);
  EXPECT_DOUBLE_EQ(calibratedForwardGaitCommand(.165, .30, .165, .30), .30);
  EXPECT_DOUBLE_EQ(calibratedForwardGaitCommand(.50, .30, .165, .30), .30);
  EXPECT_DOUBLE_EQ(calibratedForwardGaitCommand(.10, 0.0, 0.0, .75), .10);
}

#include <cstdint>
#include <limits>

#include <plan_manage/replan_fsm_utils.h>

namespace scan_planner
{

TEST(TrackingDeviation, NormalTrajectoryRequestsRollingReplan)
{
  EXPECT_EQ(classifyTrackingDeviation(
                false, false, 0.11, 0.10, 0.30,
                0.55, 1.0, 2.0, 0.0, 0.05),
            TrackingDeviationAction::ROLLING_REPLAN);
}

TEST(TrackingDeviation, NoProgressIsNotHiddenBySmallReferenceError)
{
  EXPECT_EQ(classifyTrackingDeviation(
                true, false, .01, .10, .30, 2.1, 1.0, 2.0, 0.0, .05),
            TrackingDeviationAction::LOCAL_REPAIR);
}

TEST(TrackingDeviation, ClearanceLimitedTrajectoryKeepsExecutionOwnership)
{
  EXPECT_EQ(classifyTrackingDeviation(
                true, false, 0.11, 0.10, 0.30,
                0.55, 1.0, 2.0, 0.0, 0.05),
            TrackingDeviationAction::KEEP_EXECUTING);
  EXPECT_EQ(classifyTrackingDeviation(
                true, false, 0.14, 0.10, 0.30,
                3.0, 1.0, 2.0, 0.20, 0.05),
            TrackingDeviationAction::ROLLING_REPLAN);
}

TEST(TrackingDeviation, ClearanceLimitedNoProgressEscalatesOnce)
{
  EXPECT_EQ(classifyTrackingDeviation(
                true, false, 0.12, 0.10, 0.30,
                1.99, 1.0, 2.0, 0.0, 0.05),
            TrackingDeviationAction::KEEP_EXECUTING);
  EXPECT_EQ(classifyTrackingDeviation(
                true, false, 0.12, 0.10, 0.30,
                2.0, 1.0, 2.0, 0.0, 0.05),
            TrackingDeviationAction::LOCAL_REPAIR);
}

TEST(TrackingDeviation, HarderDeviationBypassesExecutionOwnership)
{
  EXPECT_EQ(classifyTrackingDeviation(
                true, true, 0.12, 0.10, 0.30,
                0.2, 1.0, 2.0, 0.0, 0.05),
            TrackingDeviationAction::LOCAL_REPAIR);
  EXPECT_EQ(classifyTrackingDeviation(
                true, false, 0.31, 0.10, 0.30,
                0.2, 1.0, 2.0, 0.0, 0.05),
            TrackingDeviationAction::LOCAL_REPAIR);
}

TEST(PredictedCollisionInterval, ExpandsRiskWindowForDynamicHandoff)
{
  const auto interval = makePredictedCollisionInterval(
      7, 12, 42, CollisionRiskKind::HARD_MARGIN,
      5.0, 6.0, 1.0, 0.5, 0.4, 0.2, 0.15, 2.0, 10.0);

  ASSERT_TRUE(interval.valid);
  EXPECT_EQ(interval.request_id, 7U);
  EXPECT_EQ(interval.trajectory_id, 12);
  EXPECT_EQ(interval.map_revision, 42U);
  EXPECT_NEAR(interval.entry_time, 5.0, 1e-9);
  EXPECT_NEAR(interval.exit_time, 6.0, 1e-9);
  EXPECT_NEAR(interval.repair_start_time, 2.4, 1e-9);
  EXPECT_NEAR(interval.rejoin_time, 6.15, 1e-9);
}

TEST(PredictedCollisionInterval, ClampsRepairWindowToTrajectoryBounds)
{
  const auto interval = makePredictedCollisionInterval(
      1, 2, 3, CollisionRiskKind::PREFERRED_MARGIN,
      0.2, 1.0, 2.0, 2.0, 0.1, 0.1, 0.5, 0.0, 2.0);

  ASSERT_TRUE(interval.valid);
  EXPECT_DOUBLE_EQ(interval.repair_start_time, 0.0);
  EXPECT_DOUBLE_EQ(interval.rejoin_time, 1.5);
}

TEST(PredictedCollisionInterval, RejectsInvalidOrMismatchedIntervals)
{
  EXPECT_FALSE(makePredictedCollisionInterval(
      0, 1, 1, CollisionRiskKind::HARD_MARGIN,
      1.0, 2.0, 0.1, 0.1, 0.0, 0.0, 0.2, 0.0, 3.0).valid);
  EXPECT_FALSE(makePredictedCollisionInterval(
      1, 1, 1, CollisionRiskKind::HARD_MARGIN,
      2.0, 1.0, 0.1, 0.1, 0.0, 0.0, 0.2, 0.0, 3.0).valid);
  const auto valid = makePredictedCollisionInterval(
      1, 2, 3, CollisionRiskKind::HARD_MARGIN,
      1.0, 1.5, 0.1, 1.0, 0.1, 0.1, 0.2, 0.0, 3.0);
  EXPECT_FALSE(collisionIntervalMatchesExecution(valid, 1, 2, 4));
  EXPECT_TRUE(collisionIntervalMatchesExecution(valid, 1, 2, 3));
}

TEST(PredictedCollisionInterval, RequiresARejoinableWindowWithContext)
{
  auto interval = makePredictedCollisionInterval(
      7, 12, 42, CollisionRiskKind::HARD_MARGIN,
      5.0, 6.0, 0.4, 1.0, 0.2, 0.1, 0.5, 2.0, 10.0);
  interval.exit_known = true;
  interval.rejoin_position = Eigen::Vector3d(3.0, 0.0, 0.4);
  EXPECT_TRUE(localPatchSeedIsUsable(
      interval, 2.0, 10.0, 0.1, 0.5, 0.5, 42));
  EXPECT_FALSE(localPatchSeedIsUsable(
      interval, 2.0, 10.0, 0.1, 0.5, 0.5, 43));

  auto unknown_exit = interval;
  unknown_exit.exit_known = false;
  EXPECT_FALSE(localPatchSeedIsUsable(
      unknown_exit, 2.0, 10.0, 0.1, 0.5, 0.5));
  EXPECT_FALSE(localPatchSeedIsUsable(
      interval, 4.8, 10.0, 0.1, 0.5, 0.5));
  EXPECT_FALSE(localPatchSeedIsUsable(
      interval, 2.0, 6.2, 0.1, 0.5, 0.5));
}

TEST(TrajectoryHandoff, ReconcilesTimedOutHoldOnlyByExactExecutionVersion)
{
  EXPECT_EQ(reconcileTimedOutHandoff(
                true, true, true, 8, 21, 8, 21, 8, 20),
            TimedOutHandoffExecution::CANDIDATE_HELD);
  EXPECT_EQ(reconcileTimedOutHandoff(
                true, true, true, 8, 20, 8, 21, 8, 20),
            TimedOutHandoffExecution::PREVIOUS_HELD);
  EXPECT_EQ(reconcileTimedOutHandoff(
                true, true, true, 8, 19, 8, 21, 8, 20),
            TimedOutHandoffExecution::UNCONFIRMED);
  EXPECT_EQ(reconcileTimedOutHandoff(
                true, true, false, 8, 21, 8, 21, 8, 20),
            TimedOutHandoffExecution::UNCONFIRMED);
  EXPECT_EQ(reconcileTimedOutHandoff(
                false, true, true, 8, 21, 8, 21, 8, 20),
            TimedOutHandoffExecution::UNCONFIRMED);
}

TEST(TrajectoryHandoff, ReconciliationHasASecondBoundedDeadline)
{
  EXPECT_FALSE(trajectoryHandoffReconciliationTimedOut(
      false, true, 10.0, 0.75));
  EXPECT_FALSE(trajectoryHandoffReconciliationTimedOut(
      true, false, 10.0, 0.75));
  EXPECT_FALSE(trajectoryHandoffReconciliationTimedOut(
      true, true, 0.74, 0.75));
  EXPECT_TRUE(trajectoryHandoffReconciliationTimedOut(
      true, true, 0.75, 0.75));
  EXPECT_FALSE(trajectoryHandoffReconciliationTimedOut(
      true, true, std::numeric_limits<double>::quiet_NaN(), 0.75));
}

TEST(TrajectoryHandoff, ReservesAmbiguousCandidateVersionWithoutOverflow)
{
  EXPECT_EQ(reserveTrajectoryIdAfterAmbiguousHandoff(20, 21), 21);
  EXPECT_EQ(reserveTrajectoryIdAfterAmbiguousHandoff(22, 21), 22);
  EXPECT_EQ(reserveTrajectoryIdAfterAmbiguousHandoff(
                20, std::numeric_limits<int64_t>::max()),
            std::numeric_limits<int>::max());
  EXPECT_EQ(reserveTrajectoryIdAfterAmbiguousHandoff(
                20, std::numeric_limits<int64_t>::min()),
            20);
}

TEST(TrajectoryHandoff, ExhaustivelyBoundsBothAcknowledgementPhases)
{
  const std::vector<double> ages = {
      -1.0, 0.0, 0.749, 0.75, 2.0,
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()};
  for (const bool pending : {false, true})
    for (const bool hold_issued : {false, true})
      for (const double age : ages)
      {
        const bool first_phase = trajectoryHandoffTimedOut(
            pending && !hold_issued, age, 0.75);
        const bool second_phase = trajectoryHandoffReconciliationTimedOut(
            pending, hold_issued, age, 0.75);
        EXPECT_FALSE(first_phase && second_phase)
            << "pending=" << pending << " hold=" << hold_issued
            << " age=" << age;
        if (pending && std::isfinite(age) && age >= 0.75)
        {
          EXPECT_TRUE(first_phase || second_phase)
              << "pending handoff has no bounded phase at age=" << age;
        }
      }
}

TEST(LocalPatchSeed, UsesUniformTimeAndExactEndpoint)
{
  EXPECT_DOUBLE_EQ(uniformSeedSampleTime(2.0, 6.0, 0, 5), 2.0);
  EXPECT_DOUBLE_EQ(uniformSeedSampleTime(2.0, 6.0, 2, 5), 4.0);
  EXPECT_DOUBLE_EQ(uniformSeedSampleTime(2.0, 6.0, 4, 5), 6.0);
  EXPECT_TRUE(std::isnan(uniformSeedSampleTime(2.0, 6.0, 5, 5)));
}

TEST(LocalPatchSeed, RequiresFinitePointsAndMatchingEndpoints)
{
  std::vector<Eigen::Vector3d> seed;
  for (int index = 0; index < 7; ++index)
    seed.emplace_back(0.1 * index, 0.0, 0.4);
  EXPECT_TRUE(trajectorySeedMatchesBoundaries(
      seed, seed.front(), seed.back(), 1e-6));
  EXPECT_FALSE(trajectorySeedMatchesBoundaries(
      seed, seed.front(), Eigen::Vector3d(2.0, 0.0, 0.4), 0.02));
  seed[3].x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(trajectorySeedMatchesBoundaries(
      seed, seed.front(), seed.back(), 1e-6));
}

TEST(LocalPatchSeed, RequiresPositionVelocityAndAccelerationContinuity)
{
  const Eigen::Vector3d point(1.0, 2.0, 0.4);
  const Eigen::Vector3d velocity(0.3, 0.0, 0.0);
  const Eigen::Vector3d acceleration(0.0, 0.0, 0.0);
  EXPECT_TRUE(localPatchBoundaryIsContinuous(
      point, velocity, acceleration, point, velocity, acceleration,
      0.02, 0.05, 0.1));
  EXPECT_FALSE(localPatchBoundaryIsContinuous(
      point + Eigen::Vector3d(0.1, 0.0, 0.0), velocity, acceleration,
      point, velocity, acceleration, 0.02, 0.05, 0.1));
  EXPECT_FALSE(localPatchBoundaryIsContinuous(
      point, velocity + Eigen::Vector3d(0.2, 0.0, 0.0), acceleration,
      point, velocity, acceleration, 0.02, 0.05, 0.1));
}

TEST(LocalPatchSeed, RejectsStaleMapAndUnobservedExit)
{
  auto interval = makePredictedCollisionInterval(
      5, 9, 15, CollisionRiskKind::HARD_MARGIN,
      4.0, 4.5, 0.5, 1.0, 0.2, 0.1, 0.0, 1.0, 8.0, 5.0);
  interval.exit_known = true;
  interval.rejoin_position = Eigen::Vector3d(2.0, 0.0, 0.4);
  interval.entry_position = Eigen::Vector3d(1.0, 0.0, 0.4);
  interval.exit_position = Eigen::Vector3d(1.5, 0.0, 0.4);
  EXPECT_TRUE(localPatchSeedIsUsable(interval, 1.0, 8.0, 0.1, 0.3, 0.2, 15));
  EXPECT_FALSE(localPatchSeedIsUsable(interval, 1.0, 8.0, 0.1, 0.3, 0.2, 16));
  interval.exit_known = false;
  EXPECT_FALSE(localPatchSeedIsUsable(interval, 1.0, 8.0, 0.1, 0.3, 0.2, 15));
}

TEST(TrajectoryHandoff, SamplesPureRotationAndCombinedMotion)
{
  constexpr double pi = 3.14159265358979323846;
  EXPECT_EQ(planarHandoffSweepSampleCount(
                0.0, 0.0, 0.05, 5.0 * pi / 180.0), 0);
  EXPECT_EQ(planarHandoffSweepSampleCount(
                0.0, pi / 2.0, 0.05, 5.0 * pi / 180.0), 18);
  EXPECT_EQ(planarHandoffSweepSampleCount(
                0.21, 0.0, 0.05, 5.0 * pi / 180.0), 5);
  EXPECT_EQ(planarHandoffSweepSampleCount(
                0.21, pi, 0.05, 5.0 * pi / 180.0), 36);
  EXPECT_EQ(planarHandoffSweepSampleCount(
                -1.0, pi, 0.05, 5.0 * pi / 180.0), 0);
}

TEST(TrajectoryHandoff, SweepResolutionIsNeverWeakerThanEitherAxis)
{
  constexpr double pi = 3.14159265358979323846;
  constexpr double distance_step = 0.05;
  constexpr double yaw_step = 5.0 * pi / 180.0;
  for (int distance_index = 0; distance_index <= 20; ++distance_index)
    for (int degree = -180; degree <= 180; degree += 5)
    {
      const double distance = 0.025 * distance_index;
      const double yaw = degree * pi / 180.0;
      const int samples = planarHandoffSweepSampleCount(
          distance, yaw, distance_step, yaw_step);
      EXPECT_GE(samples, static_cast<int>(std::ceil(
                             distance / distance_step - 1e-12)));
      EXPECT_GE(samples, static_cast<int>(std::ceil(
                             std::abs(yaw) / yaw_step - 1e-12)));
      if (distance == 0.0 && degree != 0)
      {
        EXPECT_GT(samples, 0) << "degree=" << degree;
      }
    }
}

TEST(ReferencePathProgress, LookaheadMovesContinuouslyAlongSparseSegment)
{
  const std::vector<Eigen::Vector3d> path = {
      Eigen::Vector3d(0.0, 0.0, 0.4),
      Eigen::Vector3d(10.0, 0.0, 0.4)};

  size_t progress_segment = 0;
  double progress_ratio = 0.0;
  for (int robot_x = 1; robot_x <= 4; ++robot_x)
  {
    const auto sample = projectReferencePathLookahead(
        path, Eigen::Vector3d(robot_x, 0.0, 0.4), progress_segment, progress_ratio, 3.0);
    ASSERT_TRUE(sample.valid);
    EXPECT_NEAR(sample.projection.x(), static_cast<double>(robot_x), 1e-9);
    EXPECT_NEAR(sample.target.x(), static_cast<double>(robot_x + 3), 1e-9);
    EXPECT_NEAR(sample.target.y(), 0.0, 1e-9);
    EXPECT_GE(sample.progress_ratio, progress_ratio);
    progress_segment = sample.progress_segment;
    progress_ratio = sample.progress_ratio;
  }
}

TEST(ReferencePathProgress, ProgressDoesNotMoveBackwardWithOdometryNoise)
{
  const std::vector<Eigen::Vector3d> path = {
      Eigen::Vector3d(0.0, 0.0, 0.4),
      Eigen::Vector3d(10.0, 0.0, 0.4)};
  const auto sample = projectReferencePathLookahead(
      path, Eigen::Vector3d(3.8, 0.0, 0.4), 0, 0.4, 3.0);
  ASSERT_TRUE(sample.valid);
  EXPECT_NEAR(sample.progress_ratio, 0.4, 1e-9);
  EXPECT_NEAR(sample.target.x(), 7.0, 1e-9);
}

TEST(EmergencyRecovery, OnlyAPathReceivedWhileStoppingCanResume)
{
  EXPECT_TRUE(shouldResumePendingEmergencyPath(true, true));
  EXPECT_FALSE(shouldResumePendingEmergencyPath(false, true));
  EXPECT_FALSE(shouldResumePendingEmergencyPath(true, false));
}

TEST(SafetyRelease, NewPlanNeverClearsPhysicalPoseOccupancy)
{
  EXPECT_TRUE(newPlanMayReleaseSafetyStop(false));
  EXPECT_FALSE(newPlanMayReleaseSafetyStop(true));
  EXPECT_FALSE(newPlanMayReleaseSafetyStop(false, true, false));
  EXPECT_FALSE(newPlanMayReleaseSafetyStop(false, false, true));
}

TEST(SafetyRelease, BrakingHoldRequiresStopAndClearPlanningBelt)
{
  EXPECT_TRUE(brakingHoldMayReleaseAfterStop(
      0.0, 3, 3, false, false));
  EXPECT_FALSE(brakingHoldMayReleaseAfterStop(
      0.06, 3, 3, false, false));
  EXPECT_FALSE(brakingHoldMayReleaseAfterStop(
      0.0, 2, 3, false, false));
  EXPECT_FALSE(brakingHoldMayReleaseAfterStop(
      0.0, 3, 3, true, false));
  EXPECT_FALSE(brakingHoldMayReleaseAfterStop(
      0.0, 3, 3, false, true));
}

TEST(ClearanceEscape, AllowsOnlyNonWorseningNonPhysicalPrefix)
{
  EXPECT_TRUE(clearanceViolationAllowedDuringEscape(
      true, false, false, 0.20, 0.75, 3, 3));
  EXPECT_FALSE(clearanceViolationAllowedDuringEscape(
      true, false, true, 0.20, 0.75, 3, 3));
  EXPECT_FALSE(clearanceViolationAllowedDuringEscape(
      true, false, false, 0.20, 0.75, 4, 3));
  EXPECT_FALSE(clearanceViolationAllowedDuringEscape(
      true, true, false, 0.20, 0.75, 1, 3));
  EXPECT_FALSE(clearanceViolationAllowedDuringEscape(
      true, false, false, 0.76, 0.75, 1, 3));
  EXPECT_FALSE(clearanceViolationAllowedDuringEscape(
      false, false, false, 0.20, 0.75, 0, 0));
}

TEST(DisturbanceRecapture, HandoffUsesTheSameNonWorseningContract)
{
  EXPECT_TRUE(handoffClearanceViolationIsAdmissible(
      true, true, false, false, 3, 3));
  EXPECT_TRUE(handoffClearanceViolationIsAdmissible(
      true, true, false, false, 2, 3));
  EXPECT_FALSE(handoffClearanceViolationIsAdmissible(
      true, false, false, false, 1, 0));
  EXPECT_FALSE(handoffClearanceViolationIsAdmissible(
      true, true, false, false, 4, 3));
  EXPECT_FALSE(handoffClearanceViolationIsAdmissible(
      true, true, true, false, 1, 3));
  EXPECT_FALSE(handoffClearanceViolationIsAdmissible(
      true, true, false, true, 1, 3));
  EXPECT_FALSE(handoffClearanceViolationIsAdmissible(
      false, true, false, false, 1, 3));
}

TEST(StructuredLocalRepair, SafetyInterruptionRetriesOnlyOwnedPrimitive)
{
  constexpr uint8_t recovery_mode = 2;
  EXPECT_TRUE(structuredRepairSafetyInterruptionRequiresRetry(
      true, true, recovery_mode, recovery_mode));
  EXPECT_FALSE(structuredRepairSafetyInterruptionRequiresRetry(
      false, true, recovery_mode, recovery_mode));
  EXPECT_FALSE(structuredRepairSafetyInterruptionRequiresRetry(
      true, false, recovery_mode, recovery_mode));
  EXPECT_FALSE(structuredRepairSafetyInterruptionRequiresRetry(
      true, true, 0, recovery_mode));
}

TEST(ClearanceContract, DistinguishesPhysicalMarginAndEscapeFailures)
{
  const auto physical = classifyClearanceResult(
      true, true, false, false, false, 2);
  EXPECT_FALSE(physical.accepted());
  EXPECT_EQ(physical.failure, ClearanceFailure::PHYSICAL_COLLISION);

  const auto recoverable = classifyClearanceResult(
      true, false, true, false, false, 2);
  EXPECT_TRUE(recoverable.accepted());
  EXPECT_EQ(recoverable.failure, ClearanceFailure::NONE);

  const auto margin = classifyClearanceResult(
      true, false, false, false, false, 1);
  EXPECT_FALSE(margin.accepted());
  EXPECT_EQ(margin.failure, ClearanceFailure::PLANNING_MARGIN);

  const auto worsening = classifyClearanceResult(
      true, false, false, false, true, 3);
  EXPECT_EQ(worsening.failure, ClearanceFailure::ESCAPE_WORSENING);
}

TEST(ClearanceContract, ExhaustivelyClassifiesBooleanSafetyInputs)
{
  // Pseudo-brute-force the complete Boolean contract.  This locks down both
  // acceptance and failure precedence, so later fixes cannot make a margin
  // escape accidentally override physical collision or timeout evidence.
  for (int mask = 0; mask < 32; ++mask)
  {
    const bool margin = (mask & 1) != 0;
    const bool physical = (mask & 2) != 0;
    const bool escape_allowed = (mask & 4) != 0;
    const bool timeout = (mask & 8) != 0;
    const bool worsening = (mask & 16) != 0;
    const auto result = classifyClearanceResult(
        margin, physical, escape_allowed, timeout, worsening, 3);
    EXPECT_EQ(result.accepted(),
              !physical && !timeout && !worsening &&
                  (!margin || escape_allowed)) << "mask=" << mask;
    const ClearanceFailure expected = physical
        ? ClearanceFailure::PHYSICAL_COLLISION
        : timeout
            ? ClearanceFailure::ESCAPE_TIMEOUT
            : worsening
                ? ClearanceFailure::ESCAPE_WORSENING
                : margin && !escape_allowed
                    ? ClearanceFailure::PLANNING_MARGIN
                    : ClearanceFailure::NONE;
    EXPECT_EQ(result.failure, expected) << "mask=" << mask;
  }
}

TEST(StructuredLocalRepair, GeneratesDistinctSideAndBoundedBacktrackDirections)
{
  const auto directions = structuredLocalRepairDirections(
      Eigen::Vector2d::UnitX());
  ASSERT_EQ(directions.size(), 7U);
  EXPECT_GT(directions[0].x(), 0.0);
  EXPECT_GT(directions[0].y(), 0.0);
  EXPECT_GT(directions[1].x(), 0.0);
  EXPECT_LT(directions[1].y(), 0.0);
  EXPECT_NEAR(directions[2].dot(Eigen::Vector2d::UnitX()), 0.0, 1e-9);
  EXPECT_NEAR(directions[3].dot(Eigen::Vector2d::UnitX()), 0.0, 1e-9);
  EXPECT_LT(directions[4].x(), 0.0);
  EXPECT_GT(directions[4].y(), 0.0);
  EXPECT_LT(directions[5].x(), 0.0);
  EXPECT_LT(directions[5].y(), 0.0);
  EXPECT_NEAR(directions[6].x(), -1.0, 1e-9);
}

TEST(StructuredLocalRepair, RequiresMonotonicClearanceImprovementToFree)
{
  EXPECT_TRUE(recoveryClearanceEvidenceIsNonWorsening({3, 3, 2, 1, 0}));
  EXPECT_TRUE(recoveryClearanceEvidenceIsNonWorsening({0, 0, 0}));
  EXPECT_FALSE(recoveryClearanceEvidenceIsNonWorsening({3, 2, 3, 0}));
  EXPECT_FALSE(recoveryClearanceEvidenceIsNonWorsening({2, 1, 1}));
  EXPECT_FALSE(recoveryClearanceEvidenceIsNonWorsening({}));
}

TEST(StructuredLocalRepair, ComputesRestToRestDurationFromBothLimits)
{
  EXPECT_NEAR(restToRestMinimumDuration(0.45, 0.25, 0.50, 1.20),
              4.32, 1e-9);
  EXPECT_NEAR(restToRestMinimumDuration(0.45, 2.0, 0.50, 1.0),
              2.0 * std::sqrt(0.45 / 0.50), 1e-9);
  EXPECT_DOUBLE_EQ(restToRestMinimumDuration(0.45, 0.0, 0.50, 1.20), 0.0);
  EXPECT_DOUBLE_EQ(restToRestMinimumDuration(0.45, 0.25, 0.50, 0.9), 0.0);
}

TEST(StructuredLocalRepair, KeepsFailureOwnershipAcrossRecoveryStates)
{
  EXPECT_TRUE(localRepairOwnsFailure(true, false));
  EXPECT_TRUE(localRepairOwnsFailure(false, true));
  EXPECT_TRUE(localRepairOwnsFailure(true, true));
  EXPECT_FALSE(localRepairOwnsFailure(false, false));
}

TEST(StructuredLocalRepair, KeepsOwnershipUntilRejoinAck)
{
  EXPECT_TRUE(structuredRepairHandoffRetainsOwnership(true, false, false));
  EXPECT_TRUE(structuredRepairHandoffRetainsOwnership(true, false, true));
  EXPECT_TRUE(structuredRepairHandoffRetainsOwnership(false, true, false));
  EXPECT_FALSE(structuredRepairHandoffRetainsOwnership(false, true, true));
  EXPECT_FALSE(structuredRepairHandoffRetainsOwnership(false, false, true));

  EXPECT_FALSE(structuredRepairHandoffCompletesRecovery(true, false, true));
  EXPECT_FALSE(structuredRepairHandoffCompletesRecovery(false, true, false));
  EXPECT_TRUE(structuredRepairHandoffCompletesRecovery(false, true, true));
}

TEST(StructuredLocalRepair, SegmentBudgetSpansTheWholeTransaction)
{
  constexpr int max_segments = 5;
  for (int committed = 0; committed < max_segments; ++committed)
    EXPECT_TRUE(structuredRepairSegmentBudgetAvailable(
        committed, max_segments)) << "committed=" << committed;
  EXPECT_FALSE(structuredRepairSegmentBudgetAvailable(
      max_segments, max_segments));
  EXPECT_FALSE(structuredRepairSegmentBudgetAvailable(
      max_segments + 1, max_segments));
  EXPECT_FALSE(structuredRepairSegmentBudgetAvailable(0, 0));
  EXPECT_FALSE(structuredRepairSegmentBudgetAvailable(-1, max_segments));
}

TEST(StructuredLocalRepair, RetainsExecutionUntilExactTerminalFeedback)
{
  EXPECT_FALSE(structuredRepairTerminalFeedbackMatches(
      true, 202, 17, 202, 17, false));
  EXPECT_FALSE(structuredRepairTerminalFeedbackMatches(
      true, 202, 17, 202, 16, true));
  EXPECT_FALSE(structuredRepairTerminalFeedbackMatches(
      true, 202, 17, 201, 17, true));
  EXPECT_FALSE(structuredRepairTerminalFeedbackMatches(
      false, 202, 17, 202, 17, true));
  EXPECT_TRUE(structuredRepairTerminalFeedbackMatches(
      true, 202, 17, 202, 17, true));
}

TEST(StructuredLocalRepair, SuppressesOrdinaryRollingUntilRepairEnds)
{
  EXPECT_FALSE(ordinaryRollingReplanAllowed(true, true));
  EXPECT_FALSE(ordinaryRollingReplanAllowed(false, false));
  EXPECT_TRUE(ordinaryRollingReplanAllowed(false, true));
}

TEST(ExecutionHold, DeduplicatesOneHoldPerExecutionVersion)
{
  EXPECT_TRUE(holdCommandAlreadyIssuedForVersion(202, 17, 202, 17));
  EXPECT_FALSE(holdCommandAlreadyIssuedForVersion(202, 18, 202, 17));
  EXPECT_FALSE(holdCommandAlreadyIssuedForVersion(203, 1, 202, 17));
}

TEST(TrajectoryHandoff, RequiresBoundedControllerAcknowledgement)
{
  EXPECT_FALSE(trajectoryHandoffTimedOut(false, 10.0, 0.75));
  EXPECT_FALSE(trajectoryHandoffTimedOut(true, 0.74, 0.75));
  EXPECT_TRUE(trajectoryHandoffTimedOut(true, 0.75, 0.75));
  EXPECT_FALSE(trajectoryHandoffTimedOut(
      true, std::numeric_limits<double>::quiet_NaN(), 0.75));
  EXPECT_FALSE(trajectoryHandoffTimedOut(true, 1.0, 0.0));
}

TEST(TrajectoryHandoff, ExhaustivelyPreservesPendingAndTimeoutSemantics)
{
  const std::vector<double> elapsed = {
      -1.0, 0.0, 0.749, 0.75, 5.0,
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()};
  const std::vector<double> timeouts = {-1.0, 0.0, 0.75};
  for (const bool pending : {false, true})
    for (const double age : elapsed)
      for (const double timeout : timeouts)
      {
        const bool expected = pending && std::isfinite(age) &&
            std::isfinite(timeout) && timeout > 0.0 && age >= timeout;
        EXPECT_EQ(trajectoryHandoffTimedOut(pending, age, timeout), expected)
            << "pending=" << pending << " age=" << age
            << " timeout=" << timeout;
      }
}

TEST(TrajectoryHandoff, RejectsMalformedBsplineStructuresBeforeConstruction)
{
  EXPECT_TRUE(validBsplineStructure(6, 10, 3));
  EXPECT_FALSE(validBsplineStructure(3, 7, 3));
  EXPECT_FALSE(validBsplineStructure(6, 9, 3));
  EXPECT_FALSE(validBsplineStructure(6, 10, 0));

  EXPECT_TRUE(validBsplineKnots(
      {-0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6},
      6, 3));
  EXPECT_FALSE(validBsplineKnots(
      {-0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.2, 0.5, 0.6},
      6, 3));
  EXPECT_FALSE(validBsplineKnots(
      {-0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5,
       std::numeric_limits<double>::infinity()},
      6, 3));
  EXPECT_FALSE(validBsplineKnots(
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      6, 3));
}

TEST(ClearanceEscape, CompletesOnlyAfterActualPoseIsConfirmedFree)
{
  EXPECT_FALSE(clearanceEscapeConfirmedAtActualPose(
      true, true, true, 0.20, 0.10, 0.50, 0.15, 3, 3));
  EXPECT_FALSE(clearanceEscapeConfirmedAtActualPose(
      true, false, true, 0.20, 0.10, 0.50, 0.15, 2, 3));
  EXPECT_FALSE(clearanceEscapeConfirmedAtActualPose(
      true, false, false, 0.20, 0.10, 0.50, 0.15, 3, 3));
  EXPECT_FALSE(clearanceEscapeConfirmedAtActualPose(
      true, false, true, 0.04, 0.10, 0.50, 0.15, 3, 3));
  EXPECT_FALSE(clearanceEscapeConfirmedAtActualPose(
      true, false, true, 0.20, 0.10, 0.10, 0.15, 3, 3));
  EXPECT_TRUE(clearanceEscapeConfirmedAtActualPose(
      true, false, true, 0.20, 0.10, 0.50, 0.15, 3, 3));
  EXPECT_FALSE(clearanceEscapeConfirmedAtActualPose(
      false, false, true, 0.20, 0.10, 0.50, 0.15, 3, 3));
}

TEST(StructuredRepairTransaction, DefersReferenceUntilOwnershipEnds)
{
  EXPECT_TRUE(referenceRequestMustBeDeferred(
      true, false, false, false, false, false));
  EXPECT_TRUE(referenceRequestMustBeDeferred(
      false, true, false, false, false, false));
  EXPECT_TRUE(referenceRequestMustBeDeferred(
      false, false, true, false, false, false));
  EXPECT_TRUE(referenceRequestMustBeDeferred(
      false, false, false, true, false, false));
  EXPECT_TRUE(referenceRequestMustBeDeferred(
      false, false, false, false, true, false));
  EXPECT_TRUE(referenceRequestMustBeDeferred(
      false, false, false, false, false, true));
  EXPECT_FALSE(referenceRequestMustBeDeferred(
      false, false, false, false, false, false));
}

TEST(PredictiveBrakingSweep, IncludesReactionAndBrakingButCapsTheHorizon)
{
  EXPECT_NEAR(brakingSweepHorizon(0.75, 0.5, 0.25, 2.0), 1.75, 1e-9);
  EXPECT_NEAR(brakingSweepDistance(0.75, 0.5, 0.25, 0.25), 0.1875, 1e-9);
  EXPECT_NEAR(brakingSweepDistance(0.75, 0.5, 0.25, 1.75), 0.75, 1e-9);
  EXPECT_NEAR(brakingSweepHorizon(2.0, 0.5, 0.25, 2.0), 2.0, 1e-9);
  EXPECT_NEAR(brakingSweepHorizon(0.85, 0.32, 0.30, 4.0),
              0.30 + 0.85 / 0.32, 1e-9);
}

TEST(PredictiveBrakingSweep, FiltersGaitCycleVelocityWithoutAddingAQueue)
{
  EXPECT_NEAR(exponentialFilterAlpha(0.02, 0.20),
              1.0 - std::exp(-0.1), 1e-9);
  EXPECT_NEAR(exponentialFilterAlpha(0.02, 0.0), 1.0, 1e-9);
  EXPECT_NEAR(exponentialFilterAlpha(0.0, 0.20), 0.0, 1e-9);
}

TEST(PredictiveBrakingSweep, ConvertsChildFrameTwistToWorldBeforeFiltering)
{
  const Eigen::Quaterniond world_from_body(
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));
  const Eigen::Vector3d world_velocity = rotateBodyVelocityToWorld(
      world_from_body, Eigen::Vector3d(0.8, 0.0, 0.0));
  EXPECT_NEAR(world_velocity.x(), 0.0, 1e-9);
  EXPECT_NEAR(world_velocity.y(), 0.8, 1e-9);
}

TEST(PredictiveBrakingSweep, EstimatesWorldVelocityAndUnwrappedYawRate)
{
  const std::vector<double> times{10.0, 10.1, 10.2, 10.3};
  const std::vector<Eigen::Vector2d> positions{
      {1.0, 2.0}, {1.075, 2.04}, {1.15, 2.08}, {1.225, 2.12}};
  const double yaw0 = 3.10;
  const double yaw1 = unwrapPlanarYaw(yaw0, 3.10, -3.13);
  const double yaw2 = unwrapPlanarYaw(yaw1, -3.13, -3.08);
  const double yaw3 = unwrapPlanarYaw(yaw2, -3.08, -3.03);
  Eigen::Vector2d velocity;
  double yaw_rate = 0.0;
  ASSERT_TRUE(estimatePlanarMotionFromPoseHistory(
      times, positions, {yaw0, yaw1, yaw2, yaw3}, &velocity, &yaw_rate));
  EXPECT_NEAR(velocity.x(), 0.75, 1e-9);
  EXPECT_NEAR(velocity.y(), 0.40, 1e-9);
  EXPECT_NEAR(yaw_rate, 0.5, 0.08);
}

TEST(PredictiveBrakingSweep, TurningMotionProducesAnArcNotAWorldXAxisLine)
{
  PlanarBrakingState state;
  state.position = Eigen::Vector2d::Zero();
  state.yaw = 0.0;
  const double speed = 0.8;
  const double deceleration = 0.4;
  const double reaction = 0.2;
  const double horizon = brakingSweepHorizon(speed, deceleration, reaction, 3.0);
  double elapsed = 0.0;
  while (elapsed < horizon - 1e-9)
  {
    const double dt = std::min(0.02, horizon - elapsed);
    advancePlanarBrakingState(
        &state, speed, 0.0, 0.0, 0.5, deceleration, reaction,
        elapsed, dt);
    elapsed += dt;
  }
  EXPECT_GT(state.position.x(), 0.7);
  EXPECT_GT(state.position.y(), 0.15);
  EXPECT_GT(state.yaw, 0.25);
}

TEST(RollingReplanFailure, KeepsSafeRemainderUntilEmergencyWindow)
{
  EXPECT_TRUE(shouldKeepExecutingAfterRollingReplanFailure(
      true, false, false, false, 1.01, 1.0));
  EXPECT_FALSE(shouldKeepExecutingAfterRollingReplanFailure(
      true, false, false, false, 1.0, 1.0));
  EXPECT_FALSE(shouldKeepExecutingAfterRollingReplanFailure(
      true, false, false, false, 0.0, 1.0));
}

TEST(RollingReplanFailure, NeverBypassesReplacementOrSafetyStops)
{
  EXPECT_FALSE(shouldKeepExecutingAfterRollingReplanFailure(
      false, false, false, false, 5.0, 1.0));
  EXPECT_FALSE(shouldKeepExecutingAfterRollingReplanFailure(
      true, true, false, false, 5.0, 1.0));
  EXPECT_FALSE(shouldKeepExecutingAfterRollingReplanFailure(
      true, false, true, false, 5.0, 1.0));
  EXPECT_FALSE(shouldKeepExecutingAfterRollingReplanFailure(
      true, false, false, true, 5.0, 1.0));
}

TEST(RollingTrajectoryHandoff, RejectsStaleStartBeyondTrackingMargin)
{
  EXPECT_TRUE(isRollingTrajectoryStartFresh(0.0, 0.30));
  EXPECT_TRUE(isRollingTrajectoryStartFresh(0.30, 0.30));
  EXPECT_FALSE(isRollingTrajectoryStartFresh(0.3001, 0.30));
  EXPECT_FALSE(isRollingTrajectoryStartFresh(
      std::numeric_limits<double>::infinity(), 0.30));
}

TEST(RollingTrajectoryHandoff, TightensPositionContinuityAsSpeedIncreases)
{
  EXPECT_NEAR(speedAwareHandoffErrorLimit(
                  0.0, 0.5, 0.25, 0.10, 0.30),
              0.30, 1e-9);
  EXPECT_NEAR(speedAwareHandoffErrorLimit(
                  0.2, 0.5, 0.25, 0.10, 0.30),
              0.21, 1e-9);
  EXPECT_NEAR(speedAwareHandoffErrorLimit(
                  0.75, 0.5, 0.25, 0.10, 0.30),
              0.10, 1e-9);
}

TEST(RollingTrajectoryHandoff, StartsAtCurrentForwardPointInsteadOfReplayingFromA1)
{
  const std::vector<Eigen::Vector3d> samples = {
      Eigen::Vector3d(0.0, 0.0, 0.4),
      Eigen::Vector3d(0.2, 0.0, 0.4),
      Eigen::Vector3d(0.4, 0.0, 0.4),
      Eigen::Vector3d(0.6, 0.0, 0.4)};

  EXPECT_EQ(closestForwardTrajectorySample(
                samples, Eigen::Vector3d(0.41, 0.0, 0.4)),
            2U);
}

TEST(RollingTrajectoryHandoff, PrefersLaterSampleWhenDistanceIsTied)
{
  const std::vector<Eigen::Vector3d> samples = {
      Eigen::Vector3d(0.0, 0.0, 0.4),
      Eigen::Vector3d(0.2, 0.0, 0.4)};

  EXPECT_EQ(closestForwardTrajectorySample(
                samples, Eigen::Vector3d(0.1, 0.0, 0.4)),
            1U);
}

TEST(RollingTrajectoryHandoff, IdentifiesEmergencyStationaryTrajectory)
{
  const std::vector<Eigen::Vector3d> stationary(
      6, Eigen::Vector3d(-19.0, 12.0, 0.4));
  const std::vector<Eigen::Vector3d> moving = {
      Eigen::Vector3d(-19.0, 12.0, 0.4),
      Eigen::Vector3d(-18.9, 12.0, 0.4)};

  EXPECT_TRUE(trajectorySamplesAreStationary(stationary, 1e-4));
  EXPECT_FALSE(trajectorySamplesAreStationary(moving, 1e-4));
}

TEST(RollingTrajectoryReuse, UsesValidatedSuffixBeforeFreshPlanning)
{
  EXPECT_TRUE(shouldReuseCurrentTrajectorySuffix(
      false, true, 2.0, 0.05, 0.30));
  EXPECT_FALSE(shouldReuseCurrentTrajectorySuffix(
      true, true, 2.0, 0.05, 0.30));
  EXPECT_FALSE(shouldReuseCurrentTrajectorySuffix(
      false, false, 2.0, 0.05, 0.30));
  EXPECT_FALSE(shouldReuseCurrentTrajectorySuffix(
      false, true, 0.0, 0.05, 0.30));
  EXPECT_FALSE(shouldReuseCurrentTrajectorySuffix(
      false, true, 2.0, 0.31, 0.30));
}

TEST(StoppedLocalRepair, SafetyLatchAllowsOnlyExplicitPlanningOwners)
{
  EXPECT_FALSE(safetyLatchAllowsPlanning(false, false, false));
  EXPECT_TRUE(safetyLatchAllowsPlanning(true, false, false));
  EXPECT_TRUE(safetyLatchAllowsPlanning(false, true, false));
  EXPECT_TRUE(safetyLatchAllowsPlanning(false, false, true));
}

TEST(StoppedLocalRepair, StartsOnlyAfterAFreeStationaryPose)
{
  EXPECT_TRUE(shouldStartStoppedLocalRepair(
      true, true, 0.05, false, false));
  EXPECT_FALSE(shouldStartStoppedLocalRepair(
      true, true, 0.051, false, false));
  EXPECT_FALSE(shouldStartStoppedLocalRepair(
      true, true, 0.0, true, false));
  EXPECT_FALSE(shouldStartStoppedLocalRepair(
      true, true, 0.0, false, true));
  EXPECT_FALSE(shouldStartStoppedLocalRepair(
      true, false, 0.0, false, false));
}

TEST(RollingTrajectoryReuse, RejectsDegenerateSampledSuffix)
{
  EXPECT_FALSE(sampledSuffixIsReusable(1, 1.0));
  EXPECT_FALSE(sampledSuffixIsReusable(4, 0.0));
  EXPECT_FALSE(sampledSuffixIsReusable(4, 1e-5));
  EXPECT_TRUE(sampledSuffixIsReusable(4, 0.25));
}

TEST(RollingReplanTrigger, RequiresPhysicalProgressAsWellAsPlannedProgress)
{
  EXPECT_FALSE(shouldTriggerRollingReplan(1.10, 0.08, 1.0));
  EXPECT_FALSE(shouldTriggerRollingReplan(0.90, 1.10, 1.0));
  EXPECT_TRUE(shouldTriggerRollingReplan(1.10, 1.00, 1.0));
}

TEST(PredictiveCollision, ReplansBeforeDynamicStoppingHorizon)
{
  EXPECT_NEAR(predictiveHardStopTime(0.75, 0.5, 0.25, 0.35), 1.75, 1e-9);
  EXPECT_FALSE(predictiveCollisionRequiresHardStop(
      2.0, 0.75, 0.5, 0.25, 0.35));
  EXPECT_TRUE(predictiveCollisionRequiresHardStop(
      1.7, 0.75, 0.5, 0.25, 0.35));
  EXPECT_NEAR(predictiveHardStopTime(0.0, 0.5, 0.10, 0.35), 0.35, 1e-9);
}

TEST(RollingReplanStartVelocity, PreservesOnlyForwardPathComponent)
{
  const Eigen::Vector3d projected = projectForwardVelocityToPath(
      Eigen::Vector3d(0.6, 0.2, 0.1), Eigen::Vector3d(1.0, 0.0, 0.0));
  EXPECT_NEAR(projected.x(), 0.6, 1e-9);
  EXPECT_NEAR(projected.y(), 0.0, 1e-9);
  EXPECT_NEAR(projected.z(), 0.0, 1e-9);

  const Eigen::Vector3d backwards = projectForwardVelocityToPath(
      Eigen::Vector3d(-0.4, 0.0, 0.0), Eigen::Vector3d(1.0, 0.0, 0.0));
  EXPECT_NEAR(backwards.norm(), 0.0, 1e-9);
}

TEST(TrajectoryVersion, RejectsLateRequestAndDuplicateTrajectory)
{
  EXPECT_TRUE(shouldAcceptTrajectoryVersion(
      0, 0, 0, std::numeric_limits<int64_t>::min()));
  EXPECT_FALSE(shouldAcceptTrajectoryVersion(10, 30, 11, 31));
  EXPECT_FALSE(shouldAcceptTrajectoryVersion(11, 31, 11, 31));
  EXPECT_TRUE(shouldAcceptTrajectoryVersion(11, 32, 11, 31));
  EXPECT_TRUE(shouldAcceptTrajectoryVersion(12, 1, 11, 31));
}

TEST(TrajectoryVersion, SafetyResultFollowsExecutedTrajectoryNotNewestReference)
{
  EXPECT_TRUE(safetyResultMatchesExecution(10, 30, 10, 30));
  EXPECT_FALSE(safetyResultMatchesExecution(10, 30, 11, 31));
  EXPECT_FALSE(safetyResultMatchesExecution(11, 30, 11, 31));
}

TEST(ExecutionProtocol, RejectsLateHoldAfterNewTrajectoryWasAccepted)
{
  EXPECT_FALSE(executionCommandTargetsCurrentOrNewer(10, 30, 11, 31));
  EXPECT_FALSE(executionCommandTargetsCurrentOrNewer(11, 30, 11, 31));
  EXPECT_TRUE(executionCommandTargetsCurrentOrNewer(11, 31, 11, 31));
  EXPECT_TRUE(executionCommandTargetsCurrentOrNewer(12, 0, 11, 31));
}

TEST(ExecutionProtocol, NewTrajectoryReleasesOnlyAnOlderHold)
{
  EXPECT_TRUE(trajectorySupersedesExecutionCommand(11, 32, 11, 31));
  EXPECT_TRUE(trajectorySupersedesExecutionCommand(12, 1, 11, 31));
  EXPECT_FALSE(trajectorySupersedesExecutionCommand(11, 31, 11, 31));
  EXPECT_FALSE(trajectorySupersedesExecutionCommand(10, 99, 11, 31));
}

TEST(ExecutionProtocol, FrozenFeedbackMustMatchExactExecution)
{
  EXPECT_TRUE(executionStateMatchesExecution(11, 31, 11, 31));
  EXPECT_FALSE(executionStateMatchesExecution(10, 31, 11, 31));
  EXPECT_FALSE(executionStateMatchesExecution(11, 30, 11, 31));
}

TEST(TrackingRecovery, InterpolatesYawAcrossWrapOnTheShortestArc)
{
  const double start = 170.0 * M_PI / 180.0;
  const double finish = -170.0 * M_PI / 180.0;
  const double halfway = interpolatePlanarYaw(start, finish, 0.5);

  EXPECT_NEAR(std::abs(halfway), M_PI, 1e-9);
  EXPECT_NEAR(interpolatePlanarYaw(start, finish, 0.0), start, 1e-9);
  EXPECT_NEAR(interpolatePlanarYaw(start, finish, 1.0), finish, 1e-9);
}

TEST(TrackingRecovery, AngleNormalizationRejectsNonFiniteAndBoundsHugeValues)
{
  const double huge = normalizePlanarAngle(1e300);
  EXPECT_TRUE(std::isfinite(huge));
  EXPECT_GE(huge, -M_PI);
  EXPECT_LE(huge, M_PI);
  EXPECT_TRUE(std::isnan(normalizePlanarAngle(
      std::numeric_limits<double>::infinity())));
  EXPECT_TRUE(std::isnan(normalizePlanarAngle(
      -std::numeric_limits<double>::infinity())));
  EXPECT_TRUE(std::isnan(normalizePlanarAngle(
      std::numeric_limits<double>::quiet_NaN())));
  EXPECT_TRUE(std::isnan(interpolatePlanarYaw(
      0.0, std::numeric_limits<double>::infinity(), 0.5)));
}

TEST(OdometryValidation, NormalizesOnlyFiniteNonDegenerateQuaternions)
{
  Eigen::Quaterniond valid(2.0, 0.0, 0.0, 0.0);
  EXPECT_TRUE(normalizeFiniteQuaternion(&valid));
  EXPECT_NEAR(valid.norm(), 1.0, 1e-12);

  Eigen::Quaterniond zero(0.0, 0.0, 0.0, 0.0);
  EXPECT_FALSE(normalizeFiniteQuaternion(&zero));

  Eigen::Quaterniond non_finite(
      std::numeric_limits<double>::infinity(), 0.0, 0.0, 0.0);
  EXPECT_FALSE(normalizeFiniteQuaternion(&non_finite));
}

} // namespace scan_planner
