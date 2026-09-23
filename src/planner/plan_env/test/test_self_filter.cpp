#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cmath>

#include "plan_env/grid_map.h"

namespace
{
constexpr double kRadius = 0.35;
constexpr double kOffset = 0.18;
constexpr double kZDown = 0.45;
constexpr double kZUp = 0.20;

TEST(SelfFilterGeometry, CoversFrontAndRearDiscs)
{
  const Eigen::Vector3d center(1.0, 2.0, 0.4);
  const Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();

  EXPECT_TRUE(plan_env::pointInsideDoubleCylinder(
      Eigen::Vector3d(1.18, 2.0, 0.4), center, orientation,
      kRadius, kOffset, kZDown, kZUp));
  EXPECT_TRUE(plan_env::pointInsideDoubleCylinder(
      Eigen::Vector3d(0.82, 2.0, 0.4), center, orientation,
      kRadius, kOffset, kZDown, kZUp));
  EXPECT_FALSE(plan_env::pointInsideDoubleCylinder(
      Eigen::Vector3d(1.54, 2.0, 0.4), center, orientation,
      kRadius, kOffset, kZDown, kZUp));
}

TEST(SelfFilterGeometry, RotatesWithBodyYaw)
{
  const Eigen::Vector3d center = Eigen::Vector3d::Zero();
  const Eigen::Quaterniond orientation(
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));

  EXPECT_TRUE(plan_env::pointInsideDoubleCylinder(
      Eigen::Vector3d(0.0, 0.5, 0.0), center, orientation,
      kRadius, kOffset, kZDown, kZUp));
  EXPECT_FALSE(plan_env::pointInsideDoubleCylinder(
      Eigen::Vector3d(0.5, 0.0, 0.0), center, orientation,
      kRadius, kOffset, kZDown, kZUp));
}

TEST(SelfFilterGeometry, ClassifiesOnlyEnabledSelfHitsAsMisses)
{
  EXPECT_EQ(plan_env::endpointObservation(true, true), 0);
  EXPECT_EQ(plan_env::endpointObservation(true, false), 1);
  EXPECT_EQ(plan_env::endpointObservation(false, true), 1);
}

TEST(SelfFilterGeometry, EnforcesVerticalBounds)
{
  const Eigen::Vector3d center = Eigen::Vector3d::Zero();
  const Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();

  EXPECT_TRUE(plan_env::pointInsideDoubleCylinder(
      Eigen::Vector3d(0.0, 0.0, kZUp), center, orientation,
      kRadius, kOffset, kZDown, kZUp));
  EXPECT_FALSE(plan_env::pointInsideDoubleCylinder(
      Eigen::Vector3d(0.0, 0.0, kZUp + 0.01), center, orientation,
      kRadius, kOffset, kZDown, kZUp));
  EXPECT_FALSE(plan_env::pointInsideDoubleCylinder(
      Eigen::Vector3d(0.0, 0.0, -kZDown - 0.01), center, orientation,
      kRadius, kOffset, kZDown, kZUp));
}

TEST(TimedPoseLookup, InterpolatesPositionAndOrientation)
{
  std::deque<plan_env::TimedPose> history{
      {1000000000LL, Eigen::Vector3d(0.0, 0.0, 0.0),
       Eigen::Quaterniond::Identity()},
      {1040000000LL, Eigen::Vector3d(0.4, 0.0, 0.2),
       Eigen::Quaterniond(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()))}};
  plan_env::TimedPose result;
  const auto status = plan_env::lookupTimedPose(
      history, 1020000000LL, 50000000LL, 20000000LL, result);

  EXPECT_EQ(status, plan_env::PoseLookupResult::INTERPOLATED);
  EXPECT_NEAR(result.position.x(), 0.2, 1e-9);
  EXPECT_NEAR(result.position.z(), 0.1, 1e-9);
  const Eigen::Vector3d heading = result.orientation * Eigen::Vector3d::UnitX();
  EXPECT_NEAR(heading.x(), std::sqrt(0.5), 1e-9);
  EXPECT_NEAR(heading.y(), std::sqrt(0.5), 1e-9);
}

TEST(TimedPoseLookup, RejectsLargeGapAndStaleNearestPose)
{
  std::deque<plan_env::TimedPose> history{
      {1000000000LL, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()},
      {1100000000LL, Eigen::Vector3d::Ones(), Eigen::Quaterniond::Identity()}};
  plan_env::TimedPose result;

  EXPECT_EQ(plan_env::lookupTimedPose(
                history, 1050000000LL, 50000000LL, 20000000LL, result),
            plan_env::PoseLookupResult::GAP_TOO_LARGE);
  EXPECT_EQ(plan_env::lookupTimedPose(
                history, 1130000000LL, 50000000LL, 20000000LL, result),
            plan_env::PoseLookupResult::TOO_NEW);
}

TEST(TimedPoseLookup, AcceptsBoundedNearestAndExactPose)
{
  std::deque<plan_env::TimedPose> history{
      {1000000000LL, Eigen::Vector3d(1.0, 2.0, 3.0),
       Eigen::Quaterniond::Identity()}};
  plan_env::TimedPose result;

  EXPECT_EQ(plan_env::lookupTimedPose(
                history, 1000000000LL, 50000000LL, 20000000LL, result),
            plan_env::PoseLookupResult::EXACT);
  EXPECT_EQ(plan_env::lookupTimedPose(
                history, 1010000000LL, 50000000LL, 20000000LL, result),
            plan_env::PoseLookupResult::NEAREST);
}
}  // namespace
