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
}  // namespace
