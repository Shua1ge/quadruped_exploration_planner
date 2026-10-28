#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cmath>

#include "plan_env/grid_map.h"
#include "plan_env/raycast.h"

namespace
{
TEST(LocalMapPatch, InflatedObstacleDoesNotRequireHitAtQueryHeight)
{
  EXPECT_EQ(plan_env::localPatchOccupancy(false, true), 100);
  EXPECT_EQ(plan_env::localPatchOccupancy(true, true), 100);
  EXPECT_EQ(plan_env::localPatchOccupancy(false, false), -1);
  EXPECT_EQ(plan_env::localPatchOccupancy(true, false), 0);
}

TEST(ObservationHypothesis, UnknownRayIsNotAMotionCertificate)
{
  const Eigen::Vector3d start(0.5, 0.5, 0.5), end(5.5, 0.5, 0.5);
  // Raw query treats unknown as not known occupied, not certified free.
  EXPECT_TRUE(plan_env::observationRayMayReach(start, end, 1.,
      [](const Eigen::Vector3d&) { return 0; }));
  EXPECT_FALSE(plan_env::observationRayMayReach(start, end, 1.,
      [](const Eigen::Vector3d& p) { return p.x() > 1. && p.x() < 2. ? 1 : 0; }));
  EXPECT_FALSE(plan_env::observationRayMayReach(start, end, 1.,
      [](const Eigen::Vector3d& p) { return p.x() > 4. ? -1 : 0; }));
  // A known occupied target is an object to see, not a pose to enter.
  EXPECT_TRUE(plan_env::observationRayMayReach(start, end, 1.,
      [](const Eigen::Vector3d& p) { return p.x() > 5. ? 1 : 0; }));
}

TEST(RaycastOcclusion, ForwardTraversalCannotClearBehindFirstWall)
{
  RayCaster ray;
  ASSERT_TRUE(ray.setInput(Eigen::Vector3d(0.5, 0.5, 0.5),
                           Eigen::Vector3d(6.5, 0.5, 0.5)));
  Eigen::Vector3d cell;
  int free_observations = 0;
  int furthest_cell = -1;
  while (ray.step(cell))
  {
    furthest_cell = static_cast<int>(cell.x());
    ++free_observations;
    if (plan_env::rayTraversalStopsAtOccupied(furthest_cell == 2))
      break;
  }
  EXPECT_EQ(furthest_cell, 2);
  EXPECT_EQ(free_observations, 3);
}

TEST(RaycastTermination, NearBoundaryMovingOriginCannotOvershootCompletedAxis)
{
  // Reproduced from the isolated rolling-to-rest test's moving cloud origin.
  // Previously the traversal reached (-1, 81, 14), never the true endpoint.
  const Eigen::Vector3d start(14.000000000000002, 0., 8.);
  const Eigen::Vector3d end(4.898587196589413e-15, 80., 14.);
  RayCaster ray;
  ASSERT_TRUE(ray.setInput(start, end));
  Eigen::Vector3d cell;
  const int crossings=(end.array().floor()-start.array().floor()).abs().sum();
  int count=0;
  while (ray.step(cell)) {
    ASSERT_LT(count++, crossings);
    ASSERT_GE(cell.x(), 0.);
    ASSERT_LE(cell.y(), 80.);
    ASSERT_LE(cell.z(), 14.);
  }
  EXPECT_EQ(count, crossings);
  EXPECT_TRUE(cell.isApprox(end.array().floor().matrix()));
}

TEST(RaycastTermination, MovingPlanarScansConsumeExactlyRemainingCrossings)
{
  for (int k : {0, 1, 174, 175, 176, 350})
    for (int i=0; i<64; ++i)
      for (double z : {8., 14.}) {
        const Eigen::Vector3d start(k*.004*20, 0., 8.);
        const Eigen::Vector3d end(80.*std::cos(i*M_PI/32),
                                  80.*std::sin(i*M_PI/32), z);
        RayCaster ray;
        ASSERT_TRUE(ray.setInput(start, end));
        Eigen::Vector3d cell;
        const int crossings=(end.array().floor()-start.array().floor()).abs().sum();
        int count=0;
        while (ray.step(cell)) ASSERT_LT(count++, crossings);
        EXPECT_EQ(count, crossings);
        EXPECT_TRUE(cell.isApprox(end.array().floor().matrix()));
      }
}

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

TEST(LocalPatchHeightReference, RejectsSingleFrameVerticalJump)
{
  const double updated = plan_env::updateHeightReference(
      0.3, 0.9, 0.02, 0.75, 0.5);

  EXPECT_GT(updated, 0.3);
  EXPECT_LE(updated, 0.31);
}

TEST(LocalPatchHeightReference, FollowsSustainedSlopeWithinRateLimit)
{
  double reference = 0.3;
  for (int step = 0; step < 100; ++step)
    reference = plan_env::updateHeightReference(
        reference, 0.8, 0.02, 0.5, 0.5);

  EXPECT_GT(reference, 0.75);
  EXPECT_LT(reference, 0.81);
}

TEST(RaycastOcclusion, StopsAtHistoricalOccupiedEvidence)
{
  EXPECT_TRUE(plan_env::rayTraversalStopsAtOccupied(true));
  EXPECT_FALSE(plan_env::rayTraversalStopsAtOccupied(false));
}

TEST(RaycastOcclusion, RequiresRepeatedMissesBeforeWallDecay)
{
  EXPECT_FALSE(plan_env::occupiedMissHasEnoughConfirmation(0, 3));
  EXPECT_FALSE(plan_env::occupiedMissHasEnoughConfirmation(1, 3));
  EXPECT_FALSE(plan_env::occupiedMissHasEnoughConfirmation(2, 3));
  EXPECT_TRUE(plan_env::occupiedMissHasEnoughConfirmation(3, 3));
  EXPECT_TRUE(plan_env::occupiedMissHasEnoughConfirmation(4, 3));
  EXPECT_FALSE(plan_env::occupiedMissHasEnoughConfirmation(3, 0));
}
}  // namespace


TEST(LaserRangeEvidence, MissingInvalidAndTrueNoReturnRemainDistinct) {
  const auto miss=plan_env::laserRangeEvidence(INFINITY,.2,7.5,3.);
  ASSERT_TRUE(miss.valid);EXPECT_FALSE(miss.hit);EXPECT_DOUBLE_EQ(miss.length,3.);
  const auto hit=plan_env::laserRangeEvidence(6.,.2,7.5,3.);
  ASSERT_TRUE(hit.valid);EXPECT_TRUE(hit.hit);EXPECT_DOUBLE_EQ(hit.length,6.);
  const auto limit=plan_env::laserRangeEvidence(7.5,.2,7.5,3.);
  ASSERT_TRUE(limit.valid);EXPECT_FALSE(limit.hit);EXPECT_DOUBLE_EQ(limit.length,3.);
  for(double bad : {double(NAN),double(-INFINITY),.1,8.})
    EXPECT_FALSE(plan_env::laserRangeEvidence(bad,.2,7.5,3.).valid);
}


TEST(SelfFilterCapturePose, Mid360MountMustNotMoveTheRobotEnvelope)
{
  const Eigen::Vector3d body(0.,0.,.30), mount(.187,0.,.0803);
  const Eigen::Quaterniond pitch(Eigen::AngleAxisd(13.*M_PI/180.,Eigen::Vector3d::UnitY()));
  for (double yaw : {0.,M_PI_2,-M_PI_2}) {
    const Eigen::Quaterniond heading(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()));
    const Eigen::Vector3d rear_leg=body+heading*Eigen::Vector3d(-.38,0.,-.15);
    const Eigen::Vector3d sensor=body+heading*mount;
    EXPECT_TRUE(plan_env::pointInsideDoubleCylinder(
        rear_leg,body,heading,.25,.18,.45,.20));
    // Reproduce the old sensor-centred mask missing a valid rear-leg return.
    EXPECT_FALSE(plan_env::pointInsideDoubleCylinder(
        rear_leg,sensor,heading*pitch,.25,.18,.45,.20));
    const Eigen::Vector3d real_obstacle=body+Eigen::Vector3d(0.,0.,.24);
    EXPECT_FALSE(plan_env::pointInsideDoubleCylinder(
        real_obstacle,body,heading,.25,.18,.45,.20));
    EXPECT_TRUE(plan_env::pointInsideDoubleCylinder(
        real_obstacle,sensor,heading*pitch,.25,.18,.45,.20));
  }
}

TEST(LowReturnRouting, KeepsLowMeasurementsSeparateWithoutDeletingHigherObstacles)
{
  const Eigen::Vector3d body(0.,0.,.30);
  EXPECT_TRUE(plan_env::lowReturnOutsideBodyBand({2.,0.,0.},body,true,.25));
  EXPECT_TRUE(plan_env::lowReturnOutsideBodyBand({2.,0.,-.30},body,true,.25));
  EXPECT_FALSE(plan_env::lowReturnOutsideBodyBand({2.,0.,.08},body,true,.25));
  EXPECT_FALSE(plan_env::lowReturnOutsideBodyBand({2.,0.,.30},body,true,.25));
  EXPECT_FALSE(plan_env::lowReturnOutsideBodyBand({2.,0.,0.},body,false,.25));
  EXPECT_FALSE(plan_env::lowReturnOutsideBodyBand({2.,0.,NAN},body,true,.25));
  EXPECT_FALSE(plan_env::lowReturnOutsideBodyBand({2.,0.,0.},body,true,-.25));
  EXPECT_TRUE(plan_env::lowReturnOutsideBodyBand({2.,0.,100.},
      {0.,0.,100.30},true,.25)); // relative to the body, not world zero
}

TEST(LowReturnRouting, UsesCaptureTimeBodyHeightInsteadOfLatestOdometry)
{
  std::deque<plan_env::TimedPose> history{
      {1000000000LL,{0.,0.,.30},Eigen::Quaterniond::Identity()},
      {1040000000LL,{0.,0.,1.30},Eigen::Quaterniond::Identity()}};
  plan_env::TimedPose capture;
  ASSERT_EQ(plan_env::lookupTimedPose(history,1000000000LL,50000000LL,20000000LL,capture),
            plan_env::PoseLookupResult::EXACT);
  EXPECT_FALSE(plan_env::lowReturnOutsideBodyBand({2.,0.,.50},capture.position,true,.25));
  EXPECT_TRUE(plan_env::lowReturnOutsideBodyBand({2.,0.,.50},history.back().position,true,.25));
}

TEST(LowReturnRouting, PrefixTraversalDoesNotClearMeasuredSurfaceVoxelOrBeyond)
{
  const Eigen::Vector3d start(.187,0.,.3803), end(2.,0.,0.);
  RayCaster ray;
  ASSERT_TRUE(ray.setInput(start*20.,end*20.));
  const Eigen::Vector3d endpoint_cell=(end*20.).array().floor().matrix();
  Eigen::Vector3d cell;
  int count=0;
  while(ray.step(cell)) {
    EXPECT_FALSE(cell.isApprox(endpoint_cell));
    EXPECT_LE(cell.x(),endpoint_cell.x());
    ASSERT_LT(++count,200);
  }
  EXPECT_GT(count,0);
}

namespace {
GridMap::InflatedOccupancySnapshotPtr sourceMap(const std::vector<std::pair<Eigen::Vector3i,uint8_t>>& sources) {
  auto map=std::make_shared<GridMap::InflatedOccupancySnapshot>();
  map->resolution_inv=20.;map->body_radius=.25;map->body_offset=.18;
  map->bound_min=Eigen::Vector3i::Constant(-40);map->bound_max=Eigen::Vector3i::Constant(40);
  map->voxel_num=Eigen::Vector3i::Constant(81);map->buffer.resize(81*81*81);
  map->inflation_min={-5,-5,-8};map->inflation_max={5,5,2};
  for(int x=-5;x<=5;++x) for(int y=-5;y<=5;++y) for(int z=-8;z<=2;++z) {
    if(x*x+y*y>25) continue;
    map->inflation_offsets.insert({x,y,z});
  }
  for(const auto& item:sources) {
    const auto& source=item.first;
    map->sources[{source.x(),source.y(),source.z()}].layers=item.second;
    for(const auto& offset:map->inflation_offsets) {
      Eigen::Vector3i id=source+Eigen::Vector3i(offset[0],offset[1],offset[2]);
      if((id.array()<map->bound_min.array()).any() || (id.array()>map->bound_max.array()).any()) continue;
      id=(id.array()+81).matrix();id=id.unaryExpr([](int x){return x%81;});
      map->buffer[id.x()*81*81+id.y()*81+id.z()]=1;
    }
  }
  return map;
}
}
TEST(MotionEvidence, SourceIdentityMatchesDiscreteInflationIncludingNegativeCoordinates) {
  const auto map=sourceMap({{{0,6,0},1},{{-5,-7,-1},2},{{10,12,4},3}});
  for(int x=-10;x<12;++x) for(int y=-12;y<15;++y) for(double yaw:{0.,.7,3.}) {
    const Eigen::Vector3d p((x+.5)*.05,(y+.5)*.05,.025);
    const auto assessment=map->assessMotion(p,yaw);
    EXPECT_FALSE(assessment.unexplained_inflation);
    EXPECT_EQ(assessment.clear(),map->getInflateOccupancy(p,yaw,{})==0);
  }
}
TEST(MotionEvidence, UnknownBoundsAndMissingSourceCannotGrantEscape) {
  auto map=sourceMap({{{3,4,0},1}});
  EXPECT_TRUE(map->assessMotion({100.,0.,0.},0.).outside_map);
  auto broken=std::const_pointer_cast<GridMap::InflatedOccupancySnapshot>(map);
  broken->sources.clear();
  EXPECT_TRUE(broken->assessMotion({.025,.025,.025},0.).unexplained_inflation);
}
TEST(MotionEvidence, ReclassificationWithdrawsOnlyItsOwnLayer) {
  auto map=std::const_pointer_cast<GridMap::InflatedOccupancySnapshot>(sourceMap({{{3,4,0},3}}));
  auto initial=map->assessMotion({.025,.025,.025},0.);
  ASSERT_EQ(initial.contacts.size(),1u);
  EXPECT_EQ(initial.contacts.begin()->second.layers,3);
  map->sources.begin()->second.layers=1;
  auto after=map->assessMotion({.025,.025,.025},0.);
  EXPECT_FALSE(after.clear());
  EXPECT_EQ(after.contacts.begin()->second.layers,1);
}
TEST(TerrainMotionPolicy, SameMeasuredTerrainDifferentExplicitCapabilities) {
  plan_env::TerrainEvidence fact{{2.,1.,-.1},.30,.5,42};
  plan_env::TerrainMotionPolicy ordinary;
  ordinary.validate();
  EXPECT_EQ(ordinary.assess(fact),plan_env::TerrainVerdict::BODY_HEIGHT_RISE);
  auto stepping=ordinary;stepping.maximum_rise=.35;stepping.validate();
  EXPECT_EQ(stepping.assess(fact),plan_env::TerrainVerdict::LEG_TERRAIN);
  EXPECT_DOUBLE_EQ(fact.measured.z(),-.1); // The policy cannot move the observation.
  EXPECT_EQ(fact.scan,42u);
  fact.ground_relative_height=-.30;
  EXPECT_EQ(ordinary.assess(fact),plan_env::TerrainVerdict::DROP);
  fact.ground_relative_height=1.2;
  EXPECT_EQ(ordinary.assess(fact),plan_env::TerrainVerdict::OVERHEAD);
  ordinary.maximum_drop=-1.;EXPECT_THROW(ordinary.validate(),std::invalid_argument);
}
