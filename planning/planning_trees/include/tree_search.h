#pragma once

#include "cone.h"
#include "path.h"
#include <cmath>
#include <vector>

typedef std::array<float, 2> Point;

struct TreeSearchParams {
  float field_of_view = 3.14;
  float distance = 10;
  int max_search_iterations = 100;
  int max_waypoints_per_path = 5;
  float waypoint_field_of_view = 3.14;
  float waypoint_distance = 10;
  int path_queue_limit = 10;
  float triangulation_radius = 1;
  float triangulation_min_cone_dist = 1;
  float triangulation_min_waypoint_dist = 1;
};

class TreeSearch {
public:
  std::vector<Cone> cones;
  std::vector<Point> waypoints;
  TreeSearchParams params;

  TreeSearch(std::vector<Cone> cones, TreeSearchParams params);

  std::tuple<Path, std::vector<Path>> getPath();

  template <typename T>
  std::vector<T> filterLocal(std::vector<T> cones, float field_of_view,
                             float distance);
  std::vector<Cone> filterLocal(std::vector<Cone> cones, float field_of_view,
                                float distance, float x, float y,
                                float heading);
  std::vector<Point> filterLocal(std::vector<Point> waypoints,
                                 float field_of_view, float distance, float x,
                                 float y, float heading);

  std::vector<Point> triangulate(std::vector<Cone> cones);
};
