#include <gtest/gtest.h>
#include <plan_manage/local_task_planner.h>
#include <plan_manage/task_protocol.h>

using namespace scan_planner;

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
