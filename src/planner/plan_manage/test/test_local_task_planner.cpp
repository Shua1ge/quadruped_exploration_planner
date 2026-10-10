#include <gtest/gtest.h>
#include <plan_manage/local_task_planner.h>
#include <plan_manage/task_protocol.h>
#include <plan_env/planar_clearance_field.h>

using namespace scan_planner;

TEST(LocalTaskPlanner, DepartureCertifiesTurnAndStoppingPrefix) {
  EXPECT_TRUE(taskDepartureCertified({0,0},0,0,.3,
      [](const auto& p,double){return std::abs(p.y())<.01 && p.x()<.31;}));
  EXPECT_FALSE(taskDepartureCertified({0,0},0,1,.3,
      [](const auto&,double yaw){return std::abs(yaw)<.2;}));
  EXPECT_FALSE(taskDepartureCertified({0,0},0,0,.3,
      [](const auto& p,double){return p.x()<.15;}));
  EXPECT_FALSE(taskDepartureCertified({0,0},0,0,-1,
      [](const auto&,double){return true;}));
  EXPECT_FALSE(taskDepartureCertified({0,0},0,0,1e300,
      [](const auto&,double){return true;}));
  EXPECT_TRUE(taskDepartureCertified({0,0},1e300,-1e300,.3,
      [](const auto&,double yaw){return std::isfinite(yaw);}));
}

TEST(LocalTaskPlanner, DepartureRejectionDoesNotRejectOtherDirection) {
  EXPECT_TRUE(taskDeparturePreviouslyRejected(.05,{0}));
  EXPECT_TRUE(taskDeparturePreviouslyRejected(2*std::acos(-1.0),{0}));
  EXPECT_FALSE(taskDeparturePreviouslyRejected(std::acos(-1.0)/2,{0}));
}

TEST(LocalTaskPlanner, NarrowDepartureLatticeFollowsMeasuredHeading) {
  const double yaw=.31;
  const Eigen::Vector2d forward(std::cos(yaw),std::sin(yaw));
  const Eigen::Vector2d normal(-forward.y(),forward.x());
  LocalTaskDomain domain; domain.route={{0,0},2*forward};
  domain.corridor_radius=.4; domain.observation_center=2*forward;
  domain.observation_radius=.4;
  const auto safe=[&](const Eigen::Vector2d& a,const Eigen::Vector2d& b) {
    return std::abs(a.dot(normal))<.03 && std::abs(b.dot(normal))<.03 &&
           b.dot(forward)>=0;
  };
  const auto candidates=selectLocalTaskCandidates(domain,{0,0},forward,safe,
      [](const auto&){return true;},.2,2,256,4,
      [&](const auto& p){return p.dot(forward)>.4;},yaw);
  ASSERT_FALSE(candidates.empty());
  EXPECT_NEAR(candidates.front().approach[1].dot(normal),0,1e-9);
}

TEST(LocalTaskPlanner, AutonomousStationArrivalEndsMotionEvenDuringHeadingAlignment) {
  EXPECT_TRUE(taskStationCaptureReady(true,true,.1));
  EXPECT_TRUE(taskStationCaptureReady(true,false,.2));
  EXPECT_FALSE(taskStationCaptureReady(true,true,.21));
  EXPECT_FALSE(taskStationCaptureReady(false,true,.1));
  EXPECT_TRUE(taskStationCaptureReady(false,false,.1));
  EXPECT_FALSE(taskStationCaptureReady(true,false,-.1));
  EXPECT_FALSE(taskStationCaptureReady(true,false,std::numeric_limits<double>::quiet_NaN()));
}

TEST(LocalTaskPlanner, RetainedGoalSurvivesMovingSearchLattice) {
  LocalTaskDomain domain; domain.route={{0,0},{3,0}};
  domain.corridor_radius=1; domain.observation_center={3,0}; domain.observation_radius=1;
  const Eigen::Vector2d goal{1.07,.03};
  for(double x : {.1,.3,.6,.8}) {
    auto candidates=selectLocalTaskCandidates(domain,{x,.04},goal,
        [](const auto&,const auto&){return true;},[](const auto&){return false;},
        .2,2,1024,8,{},0,{},&goal);
    ASSERT_FALSE(candidates.empty());
    EXPECT_NEAR((candidates.front().goal-goal).norm(),0,1e-9);
    EXPECT_NEAR((candidates.front().approach.back()-goal).norm(),0,1e-9);
  }
}

TEST(LocalTaskPlanner, RetainedGoalNeverBypassesBlockedFinalConnection) {
  LocalTaskDomain domain; domain.route={{0,0},{3,0}};
  domain.corridor_radius=1;domain.observation_center={3,0};domain.observation_radius=1;
  const Eigen::Vector2d goal{1.07,0};
  const auto candidates=selectLocalTaskCandidates(domain,{0,0},goal,
      [&](const auto&,const auto& b){return (b-goal).norm()>.05;},
      [](const auto&){return false;},.2,2,1024,8,{},0,{},&goal);
  EXPECT_TRUE(candidates.empty());
}

TEST(LocalTaskPlanner, PortalProgressRequiresOrderedMeasuredCrossing) {
  std::vector<DirectedRoutePortal> portals{{{1,0},{1,0},.3},{{2,0},{1,0},.3}};
  EXPECT_EQ(advanceRoutePortal(portals,0,{1.9,0},{2.1,0}),0u); // cannot skip first
  EXPECT_EQ(advanceRoutePortal(portals,0,{.9,.4},{1.1,.4}),0u); // outside aperture
  EXPECT_EQ(advanceRoutePortal(portals,0,{1.1,0},{.9,0}),0u); // backwards
  EXPECT_EQ(advanceRoutePortal(portals,0,{0,0},{1.1,0}),0u); // teleport/discontinuity
  EXPECT_EQ(advanceRoutePortal(portals,0,{.9,0},{1.1,0}),1u);
  EXPECT_EQ(advanceRoutePortal(portals,1,{1.9,0},{2.1,0}),2u);
  const std::vector<DirectedRoutePortal> close{{{1,0},{1,0},.3},{{1.05,0},{1,0},.3}};
  EXPECT_EQ(advanceRoutePortal(close,0,{.9,0},{1.1,0}),2u); // both measured, in order
  const std::vector<DirectedRoutePortal> wrong_order{{{1.05,0},{1,0},.3},{{1,0},{1,0},.3}};
  EXPECT_EQ(advanceRoutePortal(wrong_order,0,{.9,0},{1.1,0}),1u);
}

TEST(LocalTaskPlanner, PortalProgressWorksAroundBentRoute) {
  std::vector<DirectedRoutePortal> portals{{{2,1},{0,1},.3},{{1,2},{-1,0},.3}};
  EXPECT_EQ(advanceRoutePortal(portals,0,{2,.9},{2,1.1}),1u);
  EXPECT_EQ(advanceRoutePortal(portals,1,{1.1,2},{.9,2}),2u);
}

TEST(LocalTaskPlanner, ClearanceFieldMatchesBruteForceCellDistance) {
  const std::vector<Eigen::Vector2i> obstacles{{0,0},{4,3},{7,7}};
  PlanarClearanceField field(9,9,.1,{0,0},[&](int x,int y) {
    return std::none_of(obstacles.begin(),obstacles.end(),[&](auto p){return p.x()==x && p.y()==y;});
  });
  for(int y=0;y<8;++y) for(int x=0;x<8;++x) {
    double expected=1e9;
    for(auto p:obstacles) expected=std::min(expected,(p.cast<double>()-Eigen::Vector2d(x,y)).norm()*.1);
    EXPECT_NEAR(field.distance({(x+.5)*.1,(y+.5)*.1}),std::max(0.0,expected-.1/std::sqrt(2.0)),1e-9);
  }
  EXPECT_EQ(field.distance({-1,0}),0);
  EXPECT_EQ(field.distance({std::numeric_limits<double>::quiet_NaN(),0}),0);
}

TEST(LocalTaskPlanner, SoftClearancePrefersCenterWithoutChangingHardSafety) {
  PlanarClearanceField field(21,21,.1,{0,0},[](int x,int y){return x>0 && x<20 && y>0 && y<20;});
  EXPECT_GT(field.footprintDistance({1,1},0,.18),field.footprintDistance({1,.15},0,.18));
  EXPECT_LT(PlanarClearanceField::penalty(field.distance({1,1}),.2),
            PlanarClearanceField::penalty(field.distance({1,.15}),.2));
  EXPECT_LT(PlanarClearanceField::preferenceCost(.8,.2),PlanarClearanceField::preferenceCost(.3,.2));
}

TEST(LocalTaskPlanner, SimplificationCannotEraseClearanceDetour) {
  const std::vector<Eigen::Vector2d> path{{0,0},{0,1},{1,1},{1,0}};
  auto cost=[](const Eigen::Vector2d& a,const Eigen::Vector2d& b) {
    return (a-b).norm()*(a.y()<.1 && b.y()<.1 ? 10 : 1);
  };
  auto result=simplifyTaskApproach(path,[](auto,auto){return true;},cost);
  EXPECT_GT(result.size(),2u);
  EXPECT_EQ(result.front(),path.front()); EXPECT_EQ(result.back(),path.back());
}

TEST(LocalTaskPlanner, ShortReversePrefixCannotTurnControllerAround) {
  auto shape=taskShapeResult({{0,0},{-.03,0},{0,0},{.5,0},{1,0}},{{0,0},{1,0}});
  EXPECT_FALSE(shape.valid);
  EXPECT_STREQ(shape.reason,"departure_reversal");
  EXPECT_TRUE(taskShapeResult({{0,0},{-.5,0},{-.5,1}},
                             {{0,0},{-.5,0},{-.5,1}}).valid);
}

TEST(LocalTaskPlanner, BentCorridorProgressIsNotEndpointDirection) {
  const std::vector<Eigen::Vector2d> route{{0,0},{3,0},{1,2},{4,2}};
  const Eigen::Vector2d visited{3,0}, next{2,1}, backwards{2,0};
  EXPECT_GT(taskRouteProgress(route,next), taskRouteProgress(route,visited));
  EXPECT_LT(taskRouteProgress(route,backwards), taskRouteProgress(route,visited)-.1);
  EXPECT_LT((next-visited).dot((route.back()-route.front()).normalized()),-.1);
}

TEST(LocalTaskPlanner, ExhaustedSuffixMustRegenerateFromMeasuredMotion) {
  EXPECT_FALSE(taskSuffixHasUsefulHorizon(.02));
  EXPECT_FALSE(taskSuffixHasUsefulHorizon(0));
  EXPECT_FALSE(taskSuffixHasUsefulHorizon(.3));
  EXPECT_TRUE(taskSuffixHasUsefulHorizon(.5));
  EXPECT_FALSE(taskSuffixHasUsefulHorizon(std::numeric_limits<double>::infinity()));
}

TEST(LocalTaskPlanner, SeedStopsSmoothlyInsteadOfUniformEndpointMotion) {
  double previous=0;
  for(int i=0;i<=100;++i) {
    const double s=taskSeedArcDistance(2,0.6,0,i/100.0);
    EXPECT_GE(s,previous-1e-9); EXPECT_LE(s,2+1e-9); previous=s;
  }
  const double h=1e-5;
  EXPECT_NEAR(taskSeedArcDistance(2,.6,0,h)/h,.6,1e-4);
  EXPECT_NEAR((2-taskSeedArcDistance(2,.6,0,1-h))/h,0,1e-4);
  EXPECT_NEAR(taskSeedArcDistance(2,.6,0,1),2,1e-9);
}

TEST(LocalTaskPlanner, ShapeFailureExplainsRejectedCandidate) {
  auto result=taskShapeResult({{0,0},{.5,.3},{1,-.3},{1.5,.3},{2,0}},{{0,0},{2,0}});
  EXPECT_FALSE(result.valid); EXPECT_STREQ(result.reason,"turn");
  EXPECT_GT(result.turn,result.guide_turn+.8);
  auto deviation=taskShapeResult({{0,.5},{2,.5}},{{0,0},{2,0}});
  EXPECT_STREQ(deviation.reason,"guide_deviation");
}

TEST(LocalTaskPlanner, ShapeAllowsNecessaryCornerButRejectsShortZigzag) {
  const std::vector<Eigen::Vector2d> straight{{0,0},{2,0}};
  EXPECT_TRUE(taskShapeFollowsGuide({{0,0},{1,0.1},{2,0}},straight));
  EXPECT_FALSE(taskShapeFollowsGuide({{0,0},{.5,.3},{1,-.3},{1.5,.3},{2,0}},straight));
  EXPECT_FALSE(taskShapeFollowsGuide({{0,0},{1,0},{.5,0},{2,0}},straight));
  const std::vector<Eigen::Vector2d> corner{{0,0},{1,0},{1,1}};
  EXPECT_TRUE(taskShapeFollowsGuide(corner,corner));
  EXPECT_NEAR(taskRouteProgress(corner,{1,.5}),1.5,1e-6);
}

TEST(LocalTaskPlanner, SimplificationNeverShortcutsAcrossObstacle) {
  const std::vector<Eigen::Vector2d> path{{0,0},{.5,0},{1,0},{1,1}};
  auto open=simplifyTaskApproach(path,[](const auto&,const auto&){return true;});
  EXPECT_EQ(open.size(),2u);
  auto constrained=simplifyTaskApproach(path,[](const auto& a,const auto& b){return a.x()==b.x() || a.y()==b.y();});
  EXPECT_EQ(constrained.size(),3u);
}

TEST(LocalTaskPlanner, VisiblePassingPointDoesNotForceStop) {
  LocalTaskDomain domain;
  domain.observation_center={3,0}; domain.observation_radius=3;
  EXPECT_FALSE(taskNeedsDwell(domain,{1,0},true));
  EXPECT_TRUE(taskNeedsDwell(domain,{3,0},true));
  EXPECT_FALSE(taskNeedsDwell(domain,{3,0},false));
}

TEST(LocalTaskPlanner, MovingHandoffCannotReverseButRestCanAlign) {
  EXPECT_TRUE(taskRollingDirectionAllowed({1,0},0,0.2));
  EXPECT_FALSE(taskRollingDirectionAllowed({-1,0},0,0.2));
  EXPECT_FALSE(taskRollingDirectionAllowed({0,1},0,0.2));
  EXPECT_TRUE(taskRollingDirectionAllowed({-1,0},0,0));
}

TEST(TaskProtocol, RetirementRejectsLateCandidatesNotNewVersions) {
  EXPECT_TRUE(taskCandidateIsLive({0,0},{100,2}));
  EXPECT_FALSE(taskCandidateIsLive({0,1},{0,0}));
  EXPECT_FALSE(taskCandidateIsLive({100,0},{0,0}));
  EXPECT_FALSE(taskCandidateIsLive({99,9},{100,2}));
  EXPECT_FALSE(taskCandidateIsLive({100,2},{100,2}));
  EXPECT_TRUE(taskCandidateIsLive({100,3},{100,2}));
  EXPECT_TRUE(taskCandidateIsLive({101,1},{100,2}));
  EXPECT_TRUE(taskExecutionIsRetired({100,2},{100,2}));
  EXPECT_FALSE(taskExecutionIsRetired({101,1},{100,2}));
  EXPECT_FALSE(taskExecutionIsRetired({0,0},{100,2}));
}

TEST(LocalTaskPlanner, CannotJumpAcrossWallToEquivalentGoal) {
  LocalTaskDomain domain;
  domain.route = {{0,0},{3,0}};
  domain.observation_center = {3,0};
  domain.corridor_radius = 0.5;
  domain.observation_radius = 0.5;
  auto candidates = selectLocalTaskCandidates(domain,{0,0},{3,0},
      [](const Eigen::Vector2d& a,const Eigen::Vector2d& b) {
        return a.x()<1.0 && b.x()<1.0;
      }, [](const Eigen::Vector2d&) { return true; });
  ASSERT_FALSE(candidates.empty());
  for (const auto& c : candidates) {
    EXPECT_LT(c.goal.x(),1.0);
    EXPECT_FALSE(c.observation_station);
    EXPECT_EQ(c.approach.front(),Eigen::Vector2d(0,0));
  }
}

TEST(LocalTaskPlanner, StraightCorridorDoesNotInventAlternatingTurns) {
  LocalTaskDomain domain;
  domain.route={{0,0},{2,0}}; domain.observation_center={2,0};
  domain.corridor_radius=.8; domain.observation_radius=.8;
  auto candidates=selectLocalTaskCandidates(domain,{0,0},{2,0},
      [](const Eigen::Vector2d&,const Eigen::Vector2d&){return true;},
      [](const Eigen::Vector2d&){return true;},.2,3,1024,8,{},0.0);
  ASSERT_FALSE(candidates.empty());
  const auto& path=candidates.front().approach;
  for(size_t i=1;i<path.size();++i) {
    EXPECT_GT(path[i].x(),path[i-1].x());
    EXPECT_NEAR(path[i].y(),0.0,1e-8);
  }
}

TEST(LocalTaskPlanner, PrefersClearanceDetourToBoundaryHuggingShortestPath) {
  LocalTaskDomain domain;
  domain.route={{0,0},{2,0}}; domain.observation_center={2,0};
  domain.corridor_radius=.8; domain.observation_radius=.8;
  auto candidates=selectLocalTaskCandidates(domain,{0,0},{2,0},
      [](const Eigen::Vector2d&,const Eigen::Vector2d&){return true;},
      [](const Eigen::Vector2d&){return true;},.2,3,1024,8,{},0.0,
      [](const Eigen::Vector2d& a,const Eigen::Vector2d& b) {
        return b.x()>.2 && b.x()<1.8 && std::abs(b.y())<.3 ? 8*(b-a).norm() : 0.;
      });
  ASSERT_FALSE(candidates.empty());
  double lateral=0;
  for(const auto& p:candidates.front().approach) lateral=std::max(lateral,std::abs(p.y()));
  EXPECT_GE(lateral,.39);
}

TEST(LocalTaskPlanner, CanSelectOffPolylineWithoutLeavingTaskDomain) {
  LocalTaskDomain domain;
  domain.route = {{0,0},{2,0}};
  domain.observation_center = {2,0};
  domain.corridor_radius = 0.8;
  domain.observation_radius = 0.8;
  auto candidates = selectLocalTaskCandidates(domain,{0,0},{2,0.4},
      [](const Eigen::Vector2d&,const Eigen::Vector2d&) {return true;},
      [](const Eigen::Vector2d& p) {return p.y()>0.2;});
  ASSERT_FALSE(candidates.empty());
  EXPECT_GT(candidates.front().goal.y(),0.2);
  EXPECT_TRUE(candidates.front().observation_station);
  EXPECT_TRUE(taskDomainContains(domain,candidates.front().goal));
}

TEST(LocalTaskPlanner, BoundedSearchAndInvalidInput) {
  LocalTaskDomain domain;
  domain.observation_radius = 10;
  int checks=0;
  auto safe=[&](const Eigen::Vector2d&,const Eigen::Vector2d&) {++checks;return true;};
  auto visible=[](const Eigen::Vector2d&) {return true;};
  auto candidates=selectLocalTaskCandidates(domain,{0,0},{1,0},safe,visible,
      0.2,4,5,3);
  EXPECT_LE(checks,40);
  EXPECT_LE(candidates.size(),3u);
  EXPECT_TRUE(selectLocalTaskCandidates(domain,{0,0},{1,0},safe,visible,
      0,4,5,3).empty());
}

TEST(LocalTaskPlanner, NoReturnGateRejectsBackwardGoalsInsideObservationDisk) {
  LocalTaskDomain domain;
  domain.observation_radius=10;
  domain.has_directional_gate=true;
  domain.gate_origin={0,0}; domain.gate_direction={1,0}; domain.gate_min_progress=0;
  auto candidates=selectLocalTaskCandidates(domain,{0,0},{-2,0},
      [](const Eigen::Vector2d&,const Eigen::Vector2d&) {return true;},
      [](const Eigen::Vector2d&) {return true;});
  ASSERT_FALSE(candidates.empty());
  for(const auto& candidate:candidates) EXPECT_GE(candidate.goal.x(),0);
}

TEST(LocalTaskPlanner, ExcludedStationDoesNotPreventLeavingIt) {
  LocalTaskDomain domain;
  domain.observation_radius=10;
  auto candidates=selectLocalTaskCandidates(domain,{0,0},{0,0},
      [](const Eigen::Vector2d&,const Eigen::Vector2d&) {return true;},
      [](const Eigen::Vector2d&) {return true;},.2,4,128,8,
      [](const Eigen::Vector2d& p) {return p.norm()>.4;});
  ASSERT_FALSE(candidates.empty());
  for(const auto& candidate:candidates) EXPECT_GT(candidate.goal.norm(),.4);
}
