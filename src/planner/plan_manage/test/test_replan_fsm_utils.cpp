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

} // namespace scan_planner
