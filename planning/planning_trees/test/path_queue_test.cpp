#include "../include/path_queue.h"
#include <gtest/gtest.h>

using namespace std;

TEST(TestAddNewPath, TestDefault) {
  PathQueue path_queue = PathQueue(2);
  std::vector<Cone> cones{Cone(0, 1, 0), Cone(1, 0, 0), Cone(2, 1, 0),
                          Cone(3, 0, 1)};
  Path *path = new Path(cones);
  path->addWaypoint(1, 1);
  path->addWaypoint(2, 2);
  float path_cost = path->getCost();
  bool added_point = path_queue.addNewPath(path, path_cost);
  ASSERT_EQ(path_queue.paths.size(), 1);
  ASSERT_EQ(path_queue.costs.size(), 1);
  ASSERT_EQ(path_queue.paths[0]->waypoints.size(), 3);
  ASSERT_EQ(path_queue.costs[0], path_cost);
  ASSERT_EQ(added_point, true);

  Path path2 = path->createCopy();
  path2.addWaypoint(3, 3);
  float path2_cost = path2.getCost();
  added_point = path_queue.addNewPath(&path2, path2_cost);
  ASSERT_EQ(path_queue.paths.size(), 2);
  ASSERT_EQ(path_queue.costs.size(), 2);
  ASSERT_EQ(path_queue.paths[1]->waypoints.size(), 4);
  ASSERT_EQ(path_queue.costs[1], path2_cost);
  ASSERT_EQ(added_point, true);

  Path path3 = path->createCopy();
  path3.addWaypoint(1, 2);
  float path3_cost = path3.getCost();
  added_point = path_queue.addNewPath(&path3, path3_cost);
  ASSERT_EQ(path_queue.paths.size(), 2);
  ASSERT_EQ(path_queue.costs.size(), 2);
  ASSERT_EQ(path_queue.paths[0]->waypoints.size(), 4);
  ASSERT_EQ(path_queue.costs[0], path3_cost);
  ASSERT_EQ(added_point, true);

  Path path4 = path->createCopy();
  float path4_cost = path4.getCost();
  added_point = path_queue.addNewPath(&path4, path4_cost);
  ASSERT_EQ(path_queue.paths.size(), 2);
  ASSERT_EQ(path_queue.costs.size(), 2);
  ASSERT_EQ(added_point, false);
}

TEST(TestGetCostOrder, TestWorstCost) {
  // TODO: Test that the worst cost is returned
}

TEST(TestGetCostOrder, TestBestCost) {
  // TODO: Test that the best cost is returned
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
