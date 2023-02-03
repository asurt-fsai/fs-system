#include <gtest/gtest.h>
#include "../include/tree_search.h"

using namespace std;

TEST(TestFilterLocal, TestNoPosHeading){
    std::vector<Cone> cones = {Cone(1, 0, 0), Cone(2, 1, 0), Cone(2, -1, 0), Cone(0.1, 1, 0), Cone(-0.1, 1, 0), Cone(5, 0, 0)};
    TreeSearchParams params;
    params.field_of_view = M_PI;
    params.distance = 3;

    TreeSearch tree_search = TreeSearch(cones, params);
    EXPECT_EQ(tree_search.cones.size(), 4);
    EXPECT_FLOAT_EQ(tree_search.cones[0].x, 1);
    EXPECT_FLOAT_EQ(tree_search.cones[0].y, 0);
    EXPECT_FLOAT_EQ(tree_search.cones[1].x, 2);
    EXPECT_FLOAT_EQ(tree_search.cones[1].y, 1);
    EXPECT_FLOAT_EQ(tree_search.cones[2].x, 2);
    EXPECT_FLOAT_EQ(tree_search.cones[2].y, -1);
    EXPECT_FLOAT_EQ(tree_search.cones[3].x, 0.1);
    EXPECT_FLOAT_EQ(tree_search.cones[3].y, 1);
}

TEST(TestFilterLocal, TestPosHeading){
    std::vector<Cone> cones = {Cone(1, 0, 0), Cone(2, 1, 0), Cone(2, -1, 0), Cone(0.1, 1, 0), Cone(-0.1, 1, 0), Cone(5, 0, 0)};
    TreeSearchParams params;
    params.field_of_view = M_PI;
    params.distance = 3;

    TreeSearch tree_search = TreeSearch(cones, params);
    cones = tree_search.filterLocal(cones, params.field_of_view, params.distance, 1.9, 0.0, M_PI);
    EXPECT_EQ(cones.size(), 3);
    EXPECT_FLOAT_EQ(cones[0].x, 1);
    EXPECT_FLOAT_EQ(cones[0].y, 0);
    EXPECT_FLOAT_EQ(cones[1].x, 0.1);
    EXPECT_FLOAT_EQ(cones[1].y, 1);
    EXPECT_FLOAT_EQ(cones[2].x, -0.1);
    EXPECT_FLOAT_EQ(cones[2].y, 1);
}

TEST(TestTriangulate, Test1){
    std::vector<Cone> cones = {Cone(1, 0, 0), Cone(0.1, 1, 0), Cone(-0.1, 1, 0)};
    TreeSearchParams params;
    params.triangulation_min_cone_dist = 0.5;
    params.triangulation_min_waypoint_dist = 0.5;
    params.triangulation_radius = 0.7;

    TreeSearch tree_search = TreeSearch(cones, params);

    // Checked the following ground truth array using a plotter
    float ground_truth[13][2] = {{1.7,0},{1.49497,0.494975},{1,0.7},{0.505025,0.494975},{0.3,0},{0.505025,-0.494975},{1,-0.7},{1.49497,-0.494975},{0.594975,1.49497},{0.1,1.7},{-0.394975,1.49497},{-0.6,1},{-0.394975,0.505025}};
    for (int i = 0; i < tree_search.waypoints.size(); i++)
    {
        EXPECT_NEAR(tree_search.waypoints[i].x, ground_truth[i][0], 0.0001);
        EXPECT_NEAR(tree_search.waypoints[i].y, ground_truth[i][1], 0.0001);
    }
}



int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
