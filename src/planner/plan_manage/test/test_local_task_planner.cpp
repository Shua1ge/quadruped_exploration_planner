#include <gtest/gtest.h>
#include <plan_manage/local_task_planner.h>
#include <plan_manage/task_protocol.h>

using namespace scan_planner;

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
