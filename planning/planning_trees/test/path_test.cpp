#include <gtest/gtest.h>
#include "../include/path.h"

using namespace std;
TEST(TestNearestConeToLine, TestNoCones){
    Path path;

    Waypoint waypoint1(0, 0);
    Waypoint waypoint2(1, 1);
    std::array<float, 2> left_unit_normal = waypoint1.getLeftUnitNormal(waypoint2);
    float intercept = waypoint1.getIntercept(waypoint2);

    float dist = path.getNearestConeToLine({}, left_unit_normal, intercept, waypoint1, waypoint2);
    EXPECT_FLOAT_EQ(dist, path.ERROR_NO_DETECT);
}
TEST(TestNearestConeToLine, TestWithinBounds){
    Path path;

    Waypoint waypoint1(0, 0);
    Waypoint waypoint2(1, 1);
    std::array<float, 2> left_unit_normal = waypoint1.getLeftUnitNormal(waypoint2);
    float intercept = waypoint1.getIntercept(waypoint2);

    float dist = path.getNearestConeToLine({Cone(0, 0.5, 0), Cone(0, 1, 0)}, left_unit_normal, intercept, waypoint1, waypoint2);
    EXPECT_FLOAT_EQ(dist, sqrt(1.0/8.0));

    float dist2 = path.getNearestConeToLine({Cone(0.5, 0, 0), Cone(1, 0, 0)}, left_unit_normal, intercept, waypoint1, waypoint2);
    EXPECT_FLOAT_EQ(dist2, sqrt(1.0/8.0));
}
TEST(TestNearestConeToLine, TestOutBounds){
    Path path;

    Waypoint waypoint1(0, 0);
    Waypoint waypoint2(1, 1);
    std::array<float, 2> left_unit_normal = waypoint1.getLeftUnitNormal(waypoint2);
    float intercept = waypoint1.getIntercept(waypoint2);

    float dist = path.getNearestConeToLine({Cone(-1, 1, 0), Cone(-1.1, 1, 0)}, left_unit_normal, intercept, waypoint1, waypoint2);
    EXPECT_FLOAT_EQ(dist, sqrt(2.0));

    float dist2 = path.getNearestConeToLine({Cone(2, 3, 0), Cone(2, 3.2, 0)}, left_unit_normal, intercept, waypoint1, waypoint2);
    EXPECT_FLOAT_EQ(dist2, sqrt(2*2 + 3*3));
}
TEST(TestAddWaypoint, TestEmptyPath){
    Path path;
    EXPECT_THROW(
        path.addWaypoint(0, 0);
        , std::invalid_argument);
}

TEST(TestAddWaypoint, TestNoCones){
    std::vector<Cone> cones;
    Path path(cones);

    path.addWaypoint(1, 1);

    EXPECT_FLOAT_EQ(path.left_dists[0], path.ERROR_NO_DETECT);
    EXPECT_FLOAT_EQ(path.right_dists[0], path.ERROR_NO_DETECT);
    EXPECT_FLOAT_EQ(path.left_unit_normals[0][0], -1.0/sqrt(2));
    EXPECT_FLOAT_EQ(path.left_unit_normals[0][1], 1.0/sqrt(2));
}

TEST(TestAddWaypoint, TestSimplePath){
    std::vector<Cone> cones = {Cone(0, 0.5, 0), Cone(0.5, 0, 0)};
    Path path(cones);

    path.addWaypoint(1, 1);

    EXPECT_FLOAT_EQ(path.left_dists[0], sqrt(1.0/8.0));
    EXPECT_FLOAT_EQ(path.right_dists[0], sqrt(1.0/8.0));
    EXPECT_FLOAT_EQ(path.left_unit_normals[0][0], -1.0/sqrt(2));
    EXPECT_FLOAT_EQ(path.left_unit_normals[0][1], 1.0/sqrt(2));
}

TEST(TestAddWaypoint, TestOnlyLeftSideCones){
    std::vector<Cone> cones = {Cone(0, 0.5, 0)};
    Path path(cones);

    path.addWaypoint(1, 1);

    EXPECT_FLOAT_EQ(path.left_dists[0], sqrt(1.0/8.0));
    EXPECT_FLOAT_EQ(path.right_dists[0], path.ERROR_NO_DETECT);
    EXPECT_FLOAT_EQ(path.left_unit_normals[0][0], -1.0/sqrt(2));
    EXPECT_FLOAT_EQ(path.left_unit_normals[0][1], 1.0/sqrt(2));
}

TEST(TestAddWaypoint, TestOnlyRightSideCones){
    std::vector<Cone> cones = {Cone(0.5, 0, 0)};
    Path path(cones);

    path.addWaypoint(1, 1);

    EXPECT_FLOAT_EQ(path.left_dists[0], path.ERROR_NO_DETECT);
    EXPECT_FLOAT_EQ(path.right_dists[0], sqrt(1.0/8.0));
    EXPECT_FLOAT_EQ(path.left_unit_normals[0][0], -1.0/sqrt(2));
    EXPECT_FLOAT_EQ(path.left_unit_normals[0][1], 1.0/sqrt(2));
}

TEST(TestGetIndices, Test1){
    std::vector<Cone> cones{Cone(0, 1, 0), Cone(1, 0, 0), Cone(2, 1, 0), Cone(3, 0, 0)};
    Path path(cones);
    path.addWaypoint(1, 1);
    path.addWaypoint(2, 0);
    path.addWaypoint(3, 1);

    std::vector<int> is_cone_right = path.getIndices();
    EXPECT_EQ(is_cone_right.size(), 4);
    EXPECT_EQ(is_cone_right[0], 0);
    EXPECT_EQ(is_cone_right[1], 1);
    EXPECT_EQ(is_cone_right[2], 0);
    EXPECT_EQ(is_cone_right[3], 1);
}

TEST(TestColorCost, TestOneConeWrong){
    std::vector<Cone> cones{Cone(0, 1, 0), Cone(1, 0, 0), Cone(2, 1, 0), Cone(3, 0, 1)};
    Path path(cones);
    path.addWaypoint(1, 1);
    path.addWaypoint(2, 0);
    path.addWaypoint(3, 1);

    float color_cost = path.getColorCost();
    EXPECT_FLOAT_EQ(color_cost, -1*(3*log(0.8)+1*log(0.1)));
}

TEST(TestColorCost, TestAllConesWrong){
    std::vector<Cone> cones{Cone(0, 1, 1), Cone(1, 0, 0), Cone(2, 1, 1), Cone(3, 0, 0)};
    Path path(cones);
    path.addWaypoint(1, 1);
    path.addWaypoint(2, 0);
    path.addWaypoint(3, 1);

    float color_cost = path.getColorCost();
    EXPECT_FLOAT_EQ(color_cost, -1*(4*log(0.1)));
}

TEST(TestColorCost, TestAllConesCorrect){
    std::vector<Cone> cones{Cone(0, 1, 0), Cone(1, 0, 1), Cone(2, 1, 0), Cone(3, 0, 1)};
    Path path(cones);
    path.addWaypoint(1, 1);
    path.addWaypoint(2, 0);
    path.addWaypoint(3, 1);

    float color_cost = path.getColorCost();
    EXPECT_FLOAT_EQ(color_cost, -1*(4*log(0.8)));
}

TEST(TestAngleCost, Test1){
    std::vector<Cone> cones;
    Path path(cones);
    path.addWaypoint(1, 1);
    path.addWaypoint(2, 0);
    path.addWaypoint(3, 1);

    float angle_cost = path.getAngleCost();
    EXPECT_FLOAT_EQ(angle_cost, M_PI/2);
}

TEST(TestWidthCost, Test1){
    std::vector<Cone> cones{Cone(0, 1, 0), Cone(1, 0, 0), Cone(2, 1, 0), Cone(3, 0, 0), Cone(-1, 0, 0)};
    Path path(cones);
    path.addWaypoint(1, 1);
    path.addWaypoint(2, 0);
    path.addWaypoint(3, 1);

    float width_cost = path.getPathWidthCost();
    float cone_dist = sqrt(2)/2.0 - path.TRACKWIDTH / 2.0;
    float left_dists = cone_dist*cone_dist;
    float right_dists = cone_dist*cone_dist;
    EXPECT_FLOAT_EQ(width_cost, (left_dists + right_dists)/2.0);
}

TEST(TestGetCost, Test1){
    std::vector<Cone> cones{Cone(0, 1, 0), Cone(1, 0, 0), Cone(2, 1, 0), Cone(3, 0, 1)};
    Path path(cones);
    path.addWaypoint(1, 1);
    path.addWaypoint(2, 0);
    path.addWaypoint(3, 1);

    float cone_dist = sqrt(2)/2.0 - path.TRACKWIDTH / 2.0;
    float width_cost = cone_dist * cone_dist;
    float color_cost = -1 * (3 * log(0.8) + 1 * log(0.1));
    float angle_cost = M_PI / 2;
    float expected_cost = path.TRACKWIDTHWEIGHT * width_cost;
    expected_cost += path.COLORWEIGHT * color_cost;
    expected_cost += path.ANGLEWEIGHT * angle_cost;
    EXPECT_FLOAT_EQ(path.getCost(), expected_cost);
}

TEST(TestHasWaypoint, Test1){
    std::vector<Cone> cones{Cone(0, 1, 0), Cone(1, 0, 0), Cone(2, 1, 0), Cone(3, 0, 1)};
    Path path(cones);
    path.addWaypoint(1, 1);
    path.addWaypoint(2, 0);
    path.addWaypoint(3, 1);

    EXPECT_FALSE(path.hasWaypoint(Waypoint(0,2)));
    EXPECT_TRUE(path.hasWaypoint(Waypoint(1,1)));
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
