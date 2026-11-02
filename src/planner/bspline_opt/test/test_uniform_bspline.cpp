#include <gtest/gtest.h>

#include <Eigen/Core>
#include <bspline_opt/uniform_bspline.h>

TEST(UniformBspline, EvaluatesLinearControlPoints)
{
  Eigen::MatrixXd points = Eigen::MatrixXd::Zero(3, 6);
  for (int i = 0; i < points.cols(); ++i) points(0, i) = static_cast<double>(i);

  scan_planner::UniformBspline spline(points, 3, 1.0);
  EXPECT_NEAR(spline.evaluateDeBoorT(0.0).x(), 1.0, 1e-9);
  EXPECT_NEAR(spline.evaluateDeBoorT(2.0).x(), 3.0, 1e-9);
  EXPECT_NEAR(spline.evaluateDeBoorT(2.0).y(), 0.0, 1e-9);
}

TEST(UniformBspline, DerivativeMatchesLinearSlope)
{
  Eigen::MatrixXd points = Eigen::MatrixXd::Zero(3, 6);
  for (int i = 0; i < points.cols(); ++i) points(0, i) = static_cast<double>(i);

  auto derivative = scan_planner::UniformBspline(points, 3, 0.5).getDerivative();
  const Eigen::Vector3d velocity = derivative.evaluateDeBoorT(0.75);
  EXPECT_NEAR(velocity.x(), 2.0, 1e-9);
  EXPECT_NEAR(velocity.y(), 0.0, 1e-9);
  EXPECT_NEAR(velocity.z(), 0.0, 1e-9);
}

TEST(UniformBspline, ExactRestrictionPreservesPositionVelocityAcceleration) {
  Eigen::MatrixXd p(3,12);
  for(int i=0;i<12;++i) p.col(i)<<.2*i,std::sin(i*.4),.01*i*i;
  scan_planner::UniformBspline a(p,3,.25);
  for(auto interval: {std::make_pair(0.,2.25),std::make_pair(.13,1.71),std::make_pair(.5,2.25)}) {
    auto b=a.segment(interval.first,interval.second);ASSERT_TRUE(b);
    auto av=a.getDerivative(),bv=b->getDerivative();auto aa=av.getDerivative(),ba=bv.getDerivative();
    for(int i=0;i<=100;i++) {
      double t=(interval.second-interval.first)*i/100.;
      EXPECT_LT((a.evaluateDeBoorT(interval.first+t)-b->evaluateDeBoorT(t)).norm(),1e-9);
      EXPECT_LT((av.evaluateDeBoorT(interval.first+t)-bv.evaluateDeBoorT(t)).norm(),1e-8);
      EXPECT_LT((aa.evaluateDeBoorT(interval.first+t)-ba.evaluateDeBoorT(t)).norm(),1e-7);
    }
  }
  EXPECT_FALSE(a.segment(-.1,1));EXPECT_FALSE(a.segment(1,1));EXPECT_FALSE(a.segment(0,10));
}
TEST(UniformBspline, C2JoinRetainsOldPrefixAndIndependentNewTailExactly) {
  Eigen::MatrixXd p(3,12);
  for(int i=0;i<12;++i) p.col(i)<<.15*i,.1*std::sin(.4*i),.003*i*i;
  scan_planner::UniformBspline old(p,3,.25);
  auto prefix=old.segment(.13,.91);ASSERT_TRUE(prefix);
  auto v=prefix->getDerivative();auto a=v.getDerivative();double d=prefix->getTimeSum(),h=1.3;
  Eigen::Vector3d end=prefix->evaluateDeBoorT(d),vel=v.evaluateDeBoorT(d),acc=a.evaluateDeBoorT(d);
  Eigen::MatrixXd cp(3,4);cp.col(0)=end;cp.col(1)=end+vel*h/3.;
  cp.col(2)=end+2*vel*h/3.+acc*h*h/6.;cp.col(3)=end+Eigen::Vector3d(.8,.6,.02);
  scan_planner::UniformBspline tail(cp,3,h);Eigen::VectorXd knots(8);knots<<0,0,0,0,h,h,h,h;tail.setKnot(knots);
  auto joined=scan_planner::UniformBspline::joinC2(*prefix,tail);ASSERT_TRUE(joined);
  for(int i=0;i<=100;++i) {
    double t=d*i/100.;EXPECT_LT((joined->evaluateDeBoorT(t)-old.evaluateDeBoorT(.13+t)).norm(),1e-8);
    t=h*i/100.;EXPECT_LT((joined->evaluateDeBoorT(d+t)-tail.evaluateDeBoorT(t)).norm(),1e-8);
  }
  auto jv=joined->getDerivative();auto ja=jv.getDerivative();
  EXPECT_LT((jv.evaluateDeBoorT(d-1e-8)-jv.evaluateDeBoorT(d+1e-8)).norm(),1e-6);
  EXPECT_LT((ja.evaluateDeBoorT(d-1e-8)-ja.evaluateDeBoorT(d+1e-8)).norm(),1e-6);
  cp.col(0).x()+=.1;tail.setUniformBspline(cp,3,h);tail.setKnot(knots);
  EXPECT_FALSE(scan_planner::UniformBspline::joinC2(*prefix,tail));
}
