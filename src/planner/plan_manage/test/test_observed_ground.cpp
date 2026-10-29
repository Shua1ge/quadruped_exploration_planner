#include "plan_manage/observed_ground.hpp"
#include <gtest/gtest.h>
using namespace scan_planner;
namespace {
struct Scene {std::vector<Eigen::Vector3d> p;std::vector<uint8_t> k;
 void add(double x,double y,double z,uint8_t kind=1){p.emplace_back(x,y,z);k.push_back(kind);}
 void floor(double slope=.012){for(double x=-2.;x<=7.;x+=.25)for(double y=-2.;y<=2.;y+=.25)add(x,y,slope*x+.003*std::sin(2.*x+y));}
};
double h_at(const ObservedGround& g,size_t i){return static_cast<double>(g.heights[i]);}
}
// The estimator reports measured heights above the fitted local ground and
// marks everything else unknown. It never decides what may block the robot;
// that verdict belongs to the consumers.

TEST(ObservedGroundHeights, RisingFloorSitsAtZeroHeight) {
  Scene s;s.floor();auto r=estimateObservedGroundHeights(s.p,s.k,{0.,0.,.30});
  ASSERT_TRUE(r.valid);
  size_t known=0;for(size_t i=0;i<s.p.size();++i) if(std::isfinite(h_at(r,i))) {
    ++known;EXPECT_NEAR(h_at(r,i),0.,.04);
  }
  EXPECT_GT(known,s.p.size()*.8);
  EXPECT_NEAR(r.plane.x(),.012,.005);
}

TEST(ObservedGroundHeights, RockWallAndTrenchReportTheirMeasuredHeights) {
  Scene s;s.floor();
  s.add(1.,1.,.092);   // low rock on the floor
  const size_t rock=s.p.size()-1;
  s.add(3.,0.,.35);    // structure well above the floor
  const size_t tall=s.p.size()-1;
  const size_t trench_first=s.p.size();
  for(double x=2.;x<2.6;x+=.2)
    for(double y=-.4;y<=.4;y+=.2) s.add(x,y,-.30);   // trench bottom
  auto r=estimateObservedGroundHeights(s.p,s.k,{0.,0.,.30});
  ASSERT_TRUE(r.valid);
  EXPECT_NEAR(h_at(r,rock),.08,.02);
  EXPECT_NEAR(h_at(r,tall),.31,.02);
  size_t trench_known=0;
  for(size_t i=trench_first;i<s.p.size();++i) if(std::isfinite(h_at(r,i))) {
    ++trench_known;
    EXPECT_NEAR(h_at(r,i),-.30,.04)<<"trench="<<s.p[i].transpose();
  }
  EXPECT_GT(trench_known,0u);
}

TEST(ObservedGroundHeights, MeasuredNoReturnIsNeverGivenAHeight) {
  Scene s;s.floor();s.add(2.,0.,.024,2);
  auto r=estimateObservedGroundHeights(s.p,s.k,{0.,0.,.30});
  EXPECT_FALSE(std::isfinite(h_at(r,r.heights.size()-1)));
}

TEST(ObservedGroundHeights, InsufficientOrCollinearSupportMakesEveryHeightUnknown) {
  Scene s;for(int i=0;i<30;++i)s.add(i*.2,0.,0.);
  auto r=estimateObservedGroundHeights(s.p,s.k,{0.,0.,.30});
  EXPECT_FALSE(r.valid);
  for(double h:r.heights) EXPECT_FALSE(std::isfinite(h));
}

TEST(ObservedGroundHeights, GradualRampAheadKeepsGroundHeights) {
  for(double slope:{.20,.30,.34}) {   // 11.3, 16.7, 18.8 deg
    Scene s;s.floor(slope);
    auto r=estimateObservedGroundHeights(s.p,s.k,{0.,0.,.30});
    ASSERT_TRUE(r.valid)<<"slope="<<slope;
    size_t known=0,total=0;
    for(size_t i=0;i<s.p.size();++i)
      if(s.p[i].x()>=.5 && s.p[i].x()<=2.5) {
        ++total;
        if(std::isfinite(h_at(r,i))) {++known;EXPECT_NEAR(h_at(r,i),0.,.05)<<"slope="<<slope;}
      }
    EXPECT_GT(total,0u);
    EXPECT_GT(known,total*.8)<<"slope="<<slope;
  }
}

TEST(ObservedGroundHeights, SteepSlopeCannotBecomeGround) {
  Scene s;s.floor(std::tan(40.*std::acos(-1.)/180.));
  auto r=estimateObservedGroundHeights(s.p,s.k,{0.,0.,.30});
  EXPECT_FALSE(r.valid);
}

TEST(ObservedGroundHeights, RoughFloorKeepsHeightsSmall) {
  Scene s;std::mt19937 random(20261016);
  const auto noise=[&]{return (static_cast<double>(random()%2001)/1000.-1.)*.10;};
  for(double x=-2.;x<=7.;x+=.25)for(double y=-2.;y<=2.;y+=.25) s.add(x,y,noise());
  s.add(1.,0.,.05);   // shallow pimple: reported, not judged
  s.add(-1.,1.,.20);  // real rock: reported, not judged
  auto r=estimateObservedGroundHeights(s.p,s.k,{0.,0.,.30});
  ASSERT_TRUE(r.valid);
  EXPECT_NEAR(h_at(r,r.heights.size()-2),.05,.05);
  EXPECT_NEAR(h_at(r,r.heights.size()-1),.20,.05);
}

TEST(ObservedGroundHeights, ShallowFacetsAreReportedAsLowMeasuredHeights) {
  Scene scene;
  for(double x=-2.;x<=7.;x+=.25)for(double y=-2.;y<=2.;y+=.25) {
    const bool facet=std::fmod(x+10.,2.)<.5 && std::fmod(y+10.,2.)<.5;
    scene.add(x,y,facet?.05:0.);
  }
  auto r=estimateObservedGroundHeights(scene.p,scene.k,{0.,0.,.30});
  ASSERT_TRUE(r.valid);
  size_t facet_points=0;
  for(size_t i=0;i<scene.p.size();++i)
    if(std::abs(scene.p[i].z()-.05)<1e-12 && scene.p[i].head<2>().norm()>=.55) {
    ++facet_points;
    ASSERT_TRUE(std::isfinite(h_at(r,i)));
    EXPECT_NEAR(h_at(r,i),.05,.02);
  }
  EXPECT_GT(facet_points,20u);
}

TEST(ObservedGroundHeights, RaisedPlatformEdgeIsNotSmoothedIntoTheGround) {
  Scene scene;for(double x=-2.;x<=7.;x+=.25)for(double y=-2.;y<=2.;y+=.25)
    scene.add(x,y,x>2. ? .08 : 0.);
  auto r=estimateObservedGroundHeights(scene.p,scene.k,{0.,0.,.30});
  ASSERT_TRUE(r.valid);
  size_t edges=0;
  for(size_t i=0;i<scene.p.size();++i) if(std::abs(scene.p[i].x()-2.25)<1e-8) {
    ASSERT_TRUE(std::isfinite(h_at(r,i)));
    EXPECT_GT(h_at(r,i),.03);++edges;
  }
  EXPECT_GT(edges,0u);
}

TEST(GroundHeightWire, RoundTripsAtOneCentimetreAndClamps) {
  for(double h=-0.49;h<=1.99;h+=0.01) {
    const uint8_t v=plan_env::encodeGroundHeight(h);
    EXPECT_GE(v,plan_env::kHeightFirst);EXPECT_LE(v,plan_env::kHeightLast);
    EXPECT_NEAR(plan_env::decodeGroundHeight(v),h,.0051);
  }
  EXPECT_NEAR(plan_env::decodeGroundHeight(plan_env::encodeGroundHeight(-9.)),-.50,.0051);
  EXPECT_NEAR(plan_env::decodeGroundHeight(plan_env::encodeGroundHeight(9.)),1.99,.0051);
  EXPECT_EQ(plan_env::encodeGroundHeight(std::nan("")),plan_env::kGroundUnknown);
}

TEST(GroundHeightWire, UnknownAndReservedValuesDecodeToNaN) {
  for(uint8_t v:{0,1,2,253,254,255})
    EXPECT_FALSE(std::isfinite(plan_env::decodeGroundHeight(v)));
}

TEST(ObservedGroundHeights, BodyHeightChangesDoNotMoveMeasuredWorldFloor) {
  Scene scene;scene.floor(0.);
  ObservedGround prior;
  for(double height:{.18,.30,.46,.60}) {
    auto floor=estimateObservedGroundHeights(scene.p,scene.k,{0.,0.,height},
        ObservedGroundOptions{},prior.valid ? &prior : nullptr);
    ASSERT_TRUE(floor.valid)<<height;
    EXPECT_NEAR(floor.worldPlane().z(),0.,.015);
    EXPECT_NEAR(floor.plane.z(),-height,.015);
    prior=floor;
  }
}

TEST(ObservedGroundHeights, WorldZTranslationAndNegativeAltitudeAreInvariant) {
  Scene scene;scene.floor(.15);
  auto a=estimateObservedGroundHeights(scene.p,scene.k,{0,0,.42});
  for(auto& p:scene.p) p.z()-=5.;
  auto b=estimateObservedGroundHeights(scene.p,scene.k,{0,0,-4.58});
  ASSERT_TRUE(a.valid);ASSERT_TRUE(b.valid);
  for(size_t i=0;i<a.heights.size();++i) {
    EXPECT_EQ(std::isfinite(a.heights[i]),std::isfinite(b.heights[i]));
    if(std::isfinite(a.heights[i])) { EXPECT_NEAR(a.heights[i],b.heights[i],1e-6); }
  }
  EXPECT_NEAR(b.worldPlane().z(),a.worldPlane().z()-5.,1e-6);
}

TEST(ObservedGroundHeights, MeasurementIncludesTwentyFiveToThirtyDegreeRamps) {
  for(double degrees:{25.,28.,30.}) {
    Scene scene;double slope=std::tan(degrees*std::acos(-1.)/180.);
    for(double x=-2.;x<=7.;x+=.25) for(double y=-2.;y<=2.;y+=.25)
      scene.add(x,y,slope*x);
    auto r=estimateObservedGroundHeights(scene.p,scene.k,{0,0,.30});
    ASSERT_TRUE(r.valid)<<degrees;
    EXPECT_NEAR(r.plane.x(),slope,.005);
    size_t known=0,total=0;
    for(size_t i=0;i<scene.p.size();++i) if(scene.p[i].x()>=1.5 && scene.p[i].x()<=3.) {
      ++total;if(std::isfinite(r.heights[i])) {++known;EXPECT_NEAR(r.heights[i],0.,.03);}
    }
    EXPECT_GT(known,.8*total)<<degrees;
  }
}

TEST(ObservedGroundHeights, PriorCannotSupplyMissingOrWallOnlyGround) {
  Scene floor;floor.floor();auto prior=estimateObservedGroundHeights(floor.p,floor.k,{0,0,.3});
  ASSERT_TRUE(prior.valid);
  Scene wall;for(double y=-2;y<=2;y+=.1)for(double z=0;z<=2;z+=.1)wall.add(1,y,z);
  auto r=estimateObservedGroundHeights(wall.p,wall.k,{0,0,.45},ObservedGroundOptions{},&prior);
  EXPECT_FALSE(r.valid);
  for(double h:r.heights) EXPECT_FALSE(std::isfinite(h));
  auto empty=estimateObservedGroundHeights({}, {}, {0,0,.45},ObservedGroundOptions{},&prior);
  EXPECT_FALSE(empty.valid);
}
