#include "../include/waypoint.h"
#include <cmath>
#include <gtest/gtest.h>

using namespace std;

TEST(TestInstantiation, TestWaypointNoHeading) {
  Waypoint waypoint(1, 5);
  EXPECT_FLOAT_EQ(waypoint.heading, 0);
  EXPECT_FLOAT_EQ(waypoint.tangent, 0);
}
TEST(TestInstantiation, TestWaypointHeading) {
  Waypoint waypoint(1, 5, 3);
  EXPECT_FLOAT_EQ(waypoint.heading, 3);
  EXPECT_FLOAT_EQ(waypoint.tangent, 3);
}
TEST(TestDistance, TestDistanceTwoFloats) {
  Waypoint waypoint(0, 0);
  float distance = waypoint.getDistanceTo(3, 4);
  EXPECT_FLOAT_EQ(distance, 5);
}
TEST(TestDistance, TestDistanceWaypoint) {
  Waypoint waypoint(0, 0);
  Waypoint waypoint2(3, 4);
  float distance = waypoint.getDistanceTo(waypoint2);
  EXPECT_FLOAT_EQ(distance, 5);
}
TEST(TestHeading, TestHeadingPrev) {
  float degree_45 = M_PI / 4.0;
  Waypoint waypoint(0, 0, 0);
  Waypoint waypoint_prev(-1, -1, 0);
  waypoint.updateWaypointPrev(waypoint_prev);
  EXPECT_FLOAT_EQ(waypoint.heading, degree_45);

  Waypoint waypoint2(0, 0, 0);
  Waypoint waypoint_prev2(1, 1, 0);
  waypoint2.updateWaypointPrev(waypoint_prev2);
  EXPECT_FLOAT_EQ(waypoint2.heading, -3 * degree_45);

  Waypoint waypoint3(0, 0, 0);
  Waypoint waypoint_prev3(-1, 1, 0);
  waypoint3.updateWaypointPrev(waypoint_prev3);
  EXPECT_FLOAT_EQ(waypoint3.heading, -degree_45);

  Waypoint waypoint4(0, 0, 0);
  Waypoint waypoint_prev4(1, -1, 0);
  waypoint4.updateWaypointPrev(waypoint_prev4);
  EXPECT_FLOAT_EQ(waypoint4.heading, 3 * degree_45);
}
TEST(TestHeading, TestHeadingNext) {
  float degree_45 = M_PI / 4.0;

  Waypoint waypoint(0, 0);
  Waypoint waypoint_prev(-1, -1, 0);
  waypoint.updateWaypointPrev(waypoint_prev);

  Waypoint waypoint_next1(1, 1, 0);
  waypoint.updateWaypointNext(waypoint_next1);
  EXPECT_FLOAT_EQ(waypoint.tangent, degree_45);

  Waypoint waypoint_next2(-1, 1, 0);
  waypoint.updateWaypointNext(waypoint_next2);
  EXPECT_FLOAT_EQ(waypoint.tangent, 2 * degree_45);

  Waypoint waypoint_next3(1, -1, 0);
  waypoint.updateWaypointNext(waypoint_next3);
  EXPECT_FLOAT_EQ(waypoint.tangent, 0);

  Waypoint waypoint_next4(-1, -2, 0);
  waypoint.updateWaypointNext(waypoint_next4);
  EXPECT_FLOAT_EQ(waypoint.tangent, -2 * degree_45);
}
TEST(TestFunctions, TestLeftUnitNormal) {
  Waypoint waypoint1(0, 0);
  Waypoint waypoint2(1, 1);
  std::array<float, 2> unit_normal = waypoint1.getLeftUnitNormal(waypoint2);
  EXPECT_FLOAT_EQ(unit_normal[0], -1 / sqrt(2));
  EXPECT_FLOAT_EQ(unit_normal[1], 1 / sqrt(2));
}
TEST(TestFunctions, TestYIntercept) {
  Waypoint waypoint1(1, 1);
  Waypoint waypoint2(2, 2);
  float intercept = waypoint1.getIntercept(waypoint2);
  EXPECT_FLOAT_EQ(intercept, 0);
}
TEST(TestFunctions, TestBounds) {
  Waypoint waypoint1(1, 2);
  Waypoint waypoint2(3, 4);
  std::array<float, 4> bounds = waypoint1.getBounds(waypoint2);
  EXPECT_FLOAT_EQ(bounds[0], 1);
  EXPECT_FLOAT_EQ(bounds[1], 3);
  EXPECT_FLOAT_EQ(bounds[2], 2);
  EXPECT_FLOAT_EQ(bounds[3], 4);
}
TEST(TestFunctions, TestGetDistNearestCone) {
  std::vector<Cone> cones;
  cones.push_back(Cone(-1, -2, 0));
  cones.push_back(Cone(1, 1, 0));
  cones.push_back(Cone(2, 2, 0));

  Waypoint waypoint1(0, 0);
  float distance = waypoint1.getDistNearestCone(cones);
  EXPECT_FLOAT_EQ(distance, sqrt(2));

  Waypoint waypoint2(-0.5, -0.5);
  float distance2 = waypoint2.getDistNearestCone(cones);
  EXPECT_FLOAT_EQ(distance2, sqrt(0.5*0.5 + 1.5*1.5));
}
TEST(TestFunctions, TestGetVecTo) {
  Waypoint waypoint1(1, 2);
  std::array<float, 2> point{3, 5};
  std::array<float, 2> vector = waypoint1.getVecTo(point);
  EXPECT_FLOAT_EQ(vector[0], 2);
  EXPECT_FLOAT_EQ(vector[1], 3);
}
TEST(TestFunctions, TestEquality) {
  Waypoint waypoint1(1, 2);
  Waypoint waypoint2(3, 4);
  Waypoint waypoint3(1, 2);
  Waypoint waypoint4(1, 2, 5);
  EXPECT_FALSE(waypoint1 == waypoint2);
  EXPECT_TRUE(waypoint1 == waypoint3);
  EXPECT_TRUE(waypoint1 == waypoint4);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
