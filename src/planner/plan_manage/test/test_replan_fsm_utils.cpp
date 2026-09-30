#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

#include <plan_manage/replan_fsm_utils.h>

namespace scan_planner
{

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

TEST(ClearanceEscape, CompletesOnlyAfterActualPoseIsConfirmedFree)
{
  EXPECT_FALSE(clearanceEscapeConfirmedAtActualPose(true, true, 3, 3));
  EXPECT_FALSE(clearanceEscapeConfirmedAtActualPose(true, false, 2, 3));
  EXPECT_TRUE(clearanceEscapeConfirmedAtActualPose(true, false, 3, 3));
  EXPECT_FALSE(clearanceEscapeConfirmedAtActualPose(false, false, 3, 3));
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

} // namespace scan_planner
