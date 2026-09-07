#include <gtest/gtest.h>

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

} // namespace scan_planner
