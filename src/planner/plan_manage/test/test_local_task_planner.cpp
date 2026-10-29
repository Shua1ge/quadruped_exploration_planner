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

TEST(LocalTaskPlanner, StationArrivalDoesNotInterruptHeadingAlignment) {
  EXPECT_FALSE(taskStationCaptureReady(true,true,.1));
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

TEST(LocalTaskPlanner, SmoothingShapeMetricsCannotVetoCandidate) {
  const std::vector<Eigen::Vector2d> straight{{0,0},{2,0}};
  auto turns=taskShapeResult({{0,0},{.5,.3},{1,-.3},{1.5,.3},{2,0}},straight);
  EXPECT_TRUE(turns.valid);
  EXPECT_GT(turns.turn,turns.guide_turn+.8);
  // A wide detour can be necessary for footprint clearance. Shape assessment
  // must let the actual map/domain checks decide whether it is executable.
  auto detour=taskShapeResult({{0,0},{.3,0},{.3,1},{1.7,1},{1.7,0},{2,0}},straight);
  EXPECT_TRUE(detour.valid);
  EXPECT_GT(detour.length,1.5*detour.guide_length+.2);
  EXPECT_GT(detour.deviation,.4);
  auto projection=taskShapeResult({{0,0},{1,0},{.5,0},{2,0}},straight);
  EXPECT_TRUE(projection.valid);
  EXPECT_GT(projection.regression,.15);
  const std::vector<Eigen::Vector2d> corner{{0,0},{1,0},{1,1}};
  EXPECT_TRUE(taskShapeResult(corner,corner).valid);
  EXPECT_NEAR(taskRouteProgress(corner,{1,.5}),1.5,1e-6);
}

TEST(LocalTaskPlanner, InvalidCurveCannotReachSafetyCertification) {
  const std::vector<Eigen::Vector2d> guide{{0,0},{2,0}};
  EXPECT_FALSE(taskShapeResult({},guide).valid);
  const double nan=std::numeric_limits<double>::quiet_NaN();
  const double inf=std::numeric_limits<double>::infinity();
  EXPECT_FALSE(taskShapeResult({{0,0},{nan,0},{2,0}},guide).valid);
  EXPECT_FALSE(taskShapeResult(guide,{{0,0},{inf,0}}).valid);
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


TEST(LocalTaskPlanner, StationCanDepartByCrabWithoutTurning) {
  const auto safe=[](const Eigen::Vector2d& p,double yaw) {
    return std::abs(yaw)<.05 && std::abs(p.x())<.01 && std::abs(p.y())<.31;
  };
  EXPECT_EQ(taskStationExitMask({0,0},0,.30,.28,false,safe),0);
  const auto exits=taskStationExitMask({0,0},0,.30,.28,true,safe);
  EXPECT_TRUE(exits & (1<<8));EXPECT_TRUE(exits & (1<<9));
}

TEST(LocalTaskPlanner, StationRejectsStaticSafePoseWithNoDeparture) {
  EXPECT_EQ(taskStationExitMask({0,0},0,.30,.28,true,
      [](const Eigen::Vector2d& p,double){return p.norm()<.01;}),0);
  EXPECT_EQ(taskStationExitMask({0,0},0,.30,.28,true,
      [](const Eigen::Vector2d&,double){return false;}),0);
}

TEST(LocalTaskPlanner, RecoveryRequiresUsefulMeasuredProgressAndHeading) {
  EXPECT_FALSE(lateralRecoveryReached({0,0},{0,.15},{0,.02},0,0));
  EXPECT_TRUE(lateralRecoveryReached({0,0},{0,.15},{0,.13},0,0));
  EXPECT_FALSE(lateralRecoveryReached({0,0},{0,.15},{0,0},0,0));
  EXPECT_FALSE(lateralRecoveryReached({0,0},{0,.15},{0,.15},0,.2));
  EXPECT_FALSE(lateralRecoveryReached({0,0},{0,.15},{0,-.15},0,0));
  EXPECT_FALSE(lateralRecoveryReached({0,0},{0,.01},{0,.01},0,0));
  EXPECT_NEAR(lateralRecoveryTolerance(.15),.03,1e-9);
}

TEST(LocalTaskPlanner, RecoveryCaptureBoundsBothMeasuredAxes) {
  EXPECT_TRUE(lateralRecoveryReached({0,0},{0,.15},{.08,.14},0,0));
  EXPECT_FALSE(lateralRecoveryReached({0,0},{0,.15},{.11,.14},0,0));
  EXPECT_FALSE(lateralRecoveryReached({0,0},{0,.15},{.08,.08},0,0));
  EXPECT_TRUE(lateralRecoveryReached({0,0},{0,.15},{.08,.22},0,0));
  EXPECT_FALSE(lateralRecoveryReached({0,0},{0,.15},{.08,.26},0,0));
}

TEST(LocalTaskPlanner, RecoveryProgressUsesBodyAxesAtRotatedYaw) {
  EXPECT_TRUE(lateralRecoveryReached({0,0},{-.15,0},{-.13,0},M_PI/2,M_PI/2));
  EXPECT_FALSE(lateralRecoveryReached({0,0},{-.15,0},{0,.13},M_PI/2,M_PI/2));
}


TEST(LocalTaskPlanner, ForwardStageMovesBeforeTurningAndCoversSettling) {
  const auto safe=[](const Eigen::Vector2d& p,double yaw) {
    return std::abs(p.y())<2 && p.x()<3 && (p.x()>=1.0 || std::abs(yaw)<.05);
  };
  EXPECT_FALSE(taskDepartureCertified({0,0},0,std::acos(-1.0)/2,.2,safe));
  const auto stage=taskForwardTurnStage({0,0},0,{1.2,1.0},.2,safe);
  ASSERT_TRUE(stage); EXPECT_NEAR(stage->x(),1.2,1e-9);EXPECT_NEAR(stage->y(),0,1e-9);
  // A blocked incoming line or unknown future rotation cannot qualify.
  EXPECT_FALSE(taskForwardTurnStage({0,0},0,{1.2,1.0},.2,
      [](const auto& p,double yaw){return p.x()<.3 && std::abs(yaw)<.05;}));
  EXPECT_FALSE(taskForwardTurnStage({0,0},0,{1.2,1.0},.2,
      [](const auto&,double yaw){return std::abs(yaw)<.05;}));
  EXPECT_FALSE(taskForwardTurnStage({0,0},0,{1.2,1.0},-.2,safe));
}

TEST(LocalTaskPlanner, ForwardStageRespectsBodyAxisAndCanPrepareReverseGoal) {
  const double pi=std::acos(-1.0);
  const auto stage=taskForwardTurnStage({0,0},pi/2,{-1,0},.2,
      [pi](const auto& p,double yaw){return p.y()>=.8 || std::abs(yaw-pi/2)<.05;});
  ASSERT_TRUE(stage); EXPECT_NEAR(stage->x(),0,1e-9);EXPECT_NEAR(stage->y(),1.2,1e-9);
  const auto reverse=taskForwardTurnStage({0,0},0,{-2,0},.2,
      [](const auto& p,double yaw){return p.x()>=.8 || std::abs(yaw)<.05;});
  ASSERT_TRUE(reverse);EXPECT_NEAR(reverse->x(),1.2,1e-9);
}


TEST(LocalTaskPlanner, TerminalArrivalRequiresCoherentExitAndSettlingEnvelope) {
  const auto one_way=[](const Eigen::Vector2d& p,double yaw) {
    return p.x()>-.5 && p.x()<1. && std::abs(std::remainder(yaw,2*M_PI))<.2;
  };
  EXPECT_TRUE(taskDepartureCertified({0,0},0,0,.3,one_way));
  // A coherent forward continuation is valid even without an in-place U-turn.
  EXPECT_TRUE(taskTerminalRestartCertified({0,0},0,.3,one_way));
  EXPECT_TRUE(taskTerminalRestartCertified({0,0},0,.3,
      [](const auto& p,double){return p.norm()<.6;}));
  EXPECT_TRUE(taskTerminalRestartCertified({0,0},0,.3,
      [](const auto& p,double){return p.x()>-.35 && p.x()<.6;}));
}

TEST(LocalTaskPlanner, TerminalExitUsesOneDirectionAcrossWholeCaptureBand) {
  // Each capture pose has some exit, but there is no common action. Mixing
  // successful directions from different hypothetical poses must not certify it.
  const auto safe=[](const Eigen::Vector2d& p,double yaw) {
    if(std::abs(p.y())>.01) return false;
    if(p.x()<-.10) return std::abs(yaw)<.1;
    if(p.x()>.10) return std::abs(std::remainder(yaw-M_PI,2*M_PI))<.1;
    return true;
  };
  EXPECT_FALSE(taskTerminalExitPlan({0,0},0,.3,safe));
  EXPECT_FALSE(taskTerminalExitPlan({0,0},0,.3,
      [](const auto&,double){return false;}));
}

TEST(LocalTaskPlanner, ReturnExitPlansForwardConversionInsteadOfTurningInNarrowStation) {
  const auto safe=[](const Eigen::Vector2d& p,double yaw) {
    return p.x()>-.5 && p.x()<2. && std::abs(p.y())<.2 &&
        (p.x()>.5 || std::abs(std::remainder(yaw,2*M_PI))<.1);
  };
  const auto exit=taskTerminalExitPlan({0,0},0,.3,safe,M_PI);
  ASSERT_TRUE(exit);
  EXPECT_EQ(exit->kind,TaskDepartureKind::FORWARD_STAGE);
  // The .8 m turn fits, but its return braking prefix would re-enter the
  // narrow throat. Only the 1.2 m stage fits the complete action chain.
  EXPECT_NEAR(exit->waypoint.x(),1.2,1e-9);
  EXPECT_NEAR(exit->departure_yaw,M_PI,1e-9);
  // The actual incoming/action route is independently certified by the same model.
  const auto departure=taskPlanDeparture({0,0},0,{-2,0},.3,safe);
  ASSERT_TRUE(departure);
  EXPECT_EQ(departure->kind,TaskDepartureKind::FORWARD_STAGE);
  EXPECT_NEAR((departure->objective-Eigen::Vector2d(-2,0)).norm(),0,1e-9);
  EXPECT_FALSE(taskPlanDeparture({0,0},0,{-2,0},.3,
      [&](const auto& p,double y){return p.x()<.3 && safe(p,y);}));
}

TEST(LocalTaskPlanner, RollingTransitExtendsButObservationObjectivePersists) {
  EXPECT_FALSE(taskKeepLocalTarget(false,.6,.4,1.5));
  EXPECT_TRUE(taskKeepLocalTarget(false,2.,.4,1.5));
  EXPECT_TRUE(taskKeepLocalTarget(true,.6,.4,1.5));
  EXPECT_FALSE(taskKeepLocalTarget(true,.19,.4,1.5));
}

TEST(LocalTaskPlanner, SearchComparesRestartableTerminalBeforeNearestGoal) {
  LocalTaskDomain domain;domain.route={{0,0},{2,0}};domain.corridor_radius=1.;
  domain.observation_center={2,0};domain.observation_radius=3.;
  size_t used=0;
  const auto candidates=selectLocalTaskCandidates(domain,{0,0},{2,0},
      [](const auto&,const auto&){return true;},[](const auto&){return true;},
      .2,2.,256,8,[](const auto& p){return p.x()>.5;},0.,{},nullptr,{},
      [](const auto& p,double,bool){return p.x()<1.5;},&used);
  ASSERT_FALSE(candidates.empty());EXPECT_LT(candidates.front().goal.x(),1.5);
  EXPECT_LE(used,256u);
}

TEST(LocalTaskPlanner, CornersCannotRotateInsideTheNarrowApproach) {
  LocalTaskDomain domain;domain.route={{0,0},{1,0},{1,1}};
  domain.corridor_radius=1.;domain.observation_center={5,5};domain.observation_radius=.2;
  const auto body_safe=[](const Eigen::Vector2d& p,double yaw) {
    if(p.x()>=.7) return true;
    return std::abs(p.y())+.18*std::abs(std::sin(yaw))+.35<.45;
  };
  const auto edge=[&](const Eigen::Vector2d& a,const Eigen::Vector2d& b) {
    const double yaw=std::atan2(b.y()-a.y(),b.x()-a.x());
    for(int i=0;i<=10;++i) if(!body_safe(a+(b-a)*i/10.,yaw)) return false;
    return true;
  };
  const auto turn=[&](const auto& p,double a,double b){return taskDepartureCertified(p,a,b,0.,body_safe);};
  const auto candidates=selectLocalTaskCandidates(domain,{0,0},{1,1},edge,
      [](const auto&){return false;},.2,2.,512,8,[](const auto& p){return p.y()>.5;},0.,{},nullptr,turn);
  ASSERT_FALSE(candidates.empty());
  const auto& path=candidates.front().approach;double yaw=0.;
  for(size_t i=1;i<path.size();++i) {
    const double next=std::atan2(path[i].y()-path[i-1].y(),path[i].x()-path[i-1].x());
    EXPECT_TRUE(turn(path[i-1],yaw,next));yaw=next;
  }
}


TEST(LocalTaskPlanner, TransitSpeedUsesItsOwnFullBrakingHorizon) {
  const auto safe=[](const auto& p,double){return p.x()<.9 && std::abs(p.y())<.1;};
  EXPECT_DOUBLE_EQ(taskTransitTerminalSpeed({.2,0},0,.75,.30,.55,.32,safe),.30);
  EXPECT_DOUBLE_EQ(taskTransitTerminalSpeed({.7,0},0,.75,.30,.55,.32,safe),0.);
  EXPECT_DOUBLE_EQ(taskTransitTerminalSpeed({.2,0},0,.75,.30,.55,.32,
      [](const auto&,double){return true;}),.75);
}

TEST(LocalTaskPlanner, NarrowStraightEntryUsesSeparateWideTurnCertificate) {
  const auto incoming=[](const Eigen::Vector2d&,double yaw) {
    return std::abs(yaw)<1e-6;
  };
  const auto turning=[](const Eigen::Vector2d& p,double) {
    return p.x()>=1.0;
  };
  const auto stage=taskForwardTurnStage({0,0},0,{2,1},.2,incoming,turning);
  ASSERT_TRUE(stage);
  EXPECT_NEAR(stage->x(),1.2,1e-6);
  EXPECT_FALSE(taskForwardTurnStage({0,0},0,{2,1},.2,turning));
}

#include <plan_manage/boundary_escape.h>
namespace {
plan_env::MotionAssessment contacts(std::initializer_list<std::pair<plan_env::SourceKey,double>> items,
    uint8_t layer=1) {
  plan_env::MotionAssessment result;
  for(const auto& item:items) result.contacts[item.first]={item.first,layer,item.second,{}};
  return result;
}
}
TEST(BoundaryEscape, MonotonicExitCannotExchangeSourcesWithEqualCounts) {
  const plan_env::SourceKey a{1,2,3}, b{4,5,6};
  std::map<plan_env::SourceKey,double> remaining{{a,.08}};
  EXPECT_FALSE(advanceEscapeContacts(remaining,contacts({{b,.02}})));
  ASSERT_TRUE(advanceEscapeContacts(remaining,contacts({{a,.04}})));
  EXPECT_FALSE(advanceEscapeContacts(remaining,contacts({{a,.05}})));
  EXPECT_TRUE(advanceEscapeContacts(remaining,contacts({})));
  EXPECT_TRUE(remaining.empty());
  EXPECT_FALSE(advanceEscapeContacts(remaining,contacts({{a,.01}})));
}
TEST(BoundaryEscape, NoiseAllowanceCannotAccumulateIntoDeeperPenetration) {
  const plan_env::SourceKey a{1,2,3};
  std::map<plan_env::SourceKey,double> remaining{{a,.06}};
  EXPECT_TRUE(advanceEscapeContacts(remaining,contacts({{a,.08}}),.025));
  EXPECT_FALSE(advanceEscapeContacts(remaining,contacts({{a,.10}}),.025));
  EXPECT_DOUBLE_EQ(remaining.begin()->second,.06);
}
TEST(BoundaryEscape, TerrainOrUnknownSourceCannotBorrowGeometryGrant) {
  const plan_env::SourceKey a{1,2,3};
  std::map<plan_env::SourceKey,double> remaining{{a,.05}};
  EXPECT_FALSE(advanceEscapeContacts(remaining,contacts({{a,.01}},2)));
  auto unknown=contacts({});unknown.outside_map=true;
  EXPECT_FALSE(advanceEscapeContacts(remaining,unknown));
  unknown.outside_map=false;unknown.unexplained_inflation=true;
  EXPECT_FALSE(advanceEscapeContacts(remaining,unknown));
}
TEST(BoundaryEscape, CertificateConstrainsYawZCrossDriftOvershootAndTime) {
  BoundaryEscapeCertificate c;c.goal={0.,.3,0.};c.expires=15.;
  EXPECT_TRUE(boundaryEscapePoseAllowed(c,{0.,.15,.01},.02,1.));
  EXPECT_FALSE(boundaryEscapePoseAllowed(c,{.11,.15,0.},0.,1.));
  EXPECT_FALSE(boundaryEscapePoseAllowed(c,{0.,.15,.10},0.,1.));
  EXPECT_FALSE(boundaryEscapePoseAllowed(c,{0.,.15,0.},.11,1.));
  EXPECT_FALSE(boundaryEscapePoseAllowed(c,{0.,.41,0.},0.,1.));
  EXPECT_FALSE(boundaryEscapePoseAllowed(c,{0.,.15,0.},0.,16.));
}
TEST(BoundaryEscape, CaptureStopsBeforeEndpointUsingMeasuredBrakingDistance) {
  EXPECT_NEAR(lateralCaptureStopLead(.3,.32,.3),.230625,1e-9);
  EXPECT_DOUBLE_EQ(lateralCaptureStopLead(0.,.32,.3),0.);
  EXPECT_DOUBLE_EQ(lateralCaptureStopLead(-.3,.32,.3),0.);
  EXPECT_DOUBLE_EQ(lateralCaptureStopLead(.3,0.,.3),0.);
}

TEST(LocalTaskPlanner, RejectedSearchConsumesNoPhysicalMotionAndHasOneProbe) {
  DepartureRecoveryMemory m;
  ASSERT_TRUE(m.beginSearch(10.,{1,2},.3,1));
  EXPECT_FALSE(m.used);EXPECT_EQ(m.searches,1u);
  EXPECT_EQ(m.searchState(10.01),DepartureRecoveryMemory::Search::WAIT);
  EXPECT_FALSE(m.beginSearch(11.99,{1,2},.3,1));
  ASSERT_TRUE(m.beginSearch(12.,{1,2},.3,1));
  EXPECT_FALSE(m.used);EXPECT_EQ(m.searches,2u);
  EXPECT_EQ(m.searchState(12.01),DepartureRecoveryMemory::Search::EXHAUSTED);
  EXPECT_FALSE(m.beginSearch(100.,{1,2},.3,1));
}

TEST(LocalTaskPlanner, EpisodeCannotRefillFromTaskPoseOrClockChanges) {
  DepartureRecoveryMemory m;
  ASSERT_TRUE(m.beginSearch(0.,{1,2},.3,1));
  ASSERT_TRUE(m.beginSearch(2.,{1.4,2},1.5,1));
  EXPECT_EQ(m.searches,2u);
  EXPECT_FALSE(m.beginSearch(4.,{1,2},.3,1));
  EXPECT_TRUE(m.timedOut(15.,false));
  EXPECT_TRUE(m.timedOut(-1.,false));
}

TEST(LocalTaskPlanner, OnlyActivationConsumesPhysicalBudgetAndIsBounded) {
  DepartureRecoveryMemory m;
  ASSERT_TRUE(m.beginSearch(10.,{1,2},.3,1));
  EXPECT_FALSE(m.used);
  m.activated(11.);
  EXPECT_TRUE(m.used);
  EXPECT_FALSE(m.timedOut(20.9,true));
  EXPECT_TRUE(m.timedOut(21.,true));
  EXPECT_TRUE(m.timedOut(25.,false));
  // The second successful proposal still has time to install/execute;
  // the search counter alone must not cancel it.
  EXPECT_FALSE(m.timedOut(12.,false));
}

TEST(LocalTaskPlanner, OnlyPersistentNewPhysicalExitReopensAnEpisode) {
  DepartureRecoveryMemory m;ASSERT_TRUE(m.beginSearch(10.,{1,2},0.,1));
  EXPECT_FALSE(m.observeExits(1));
  EXPECT_FALSE(m.observeExits(3));
  EXPECT_FALSE(m.observeExits(1)); // a flickering clear query is not new evidence
  EXPECT_FALSE(m.observeExits(3));
  EXPECT_FALSE(m.observeExits(5)); // different exits cannot accumulate
  EXPECT_FALSE(m.observeExits(5));
  EXPECT_TRUE(m.observeExits(5));
  EXPECT_EQ(m.searches,1u); // the owner, not the observation method, resets it
}

TEST(LocalTaskPlanner, ActualFailedMotionDoesNotBuyAnotherEscapeOnANewGoal) {
  DepartureRecoveryMemory m;ASSERT_TRUE(m.beginSearch(10.,{1,2},0.,1));
  m.activated(11.);m.motion_failed=true;
  EXPECT_EQ(m.searches,1u);
  EXPECT_EQ(m.searchState(12.),DepartureRecoveryMemory::Search::EXHAUSTED);
  EXPECT_FALSE(m.beginSearch(12.,{1,2},0.,1));
}

TEST(LocalTaskPlanner, MeasuredSlopeAndWalkCapabilityAreSeparate) {
  for(double degrees:{0.,20.,24.,25.,28.,30.}) {
    std::vector<Eigen::Vector3d> support;
    for(double x=-.4;x<=.4;x+=.1)for(double y=-.4;y<=.4;y+=.1)
      support.emplace_back(100.+x,-20.+y,-5.+x*std::tan(degrees*std::acos(-1.)/180.));
    const auto slope=measuredSupportSlope(support);
    ASSERT_TRUE(slope);EXPECT_NEAR(*slope,degrees*std::acos(-1.)/180.,1e-8);
    EXPECT_EQ(*slope>25.*std::acos(-1.)/180.+1e-6,degrees>25.);
  }
}
TEST(LocalTaskPlanner, MissingOrCollinearSupportCannotInventSlope) {
  EXPECT_FALSE(measuredSupportSlope({}));
  std::vector<Eigen::Vector3d> line;
  for(int i=0;i<20;++i) line.emplace_back(i*.05,0.,i*.01);
  EXPECT_FALSE(measuredSupportSlope(line));
}
