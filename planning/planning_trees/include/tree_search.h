#pragma once

#include "cone.h"
#include "path.h"
#include <cmath>
#include <vector>

typedef std::array<float, 2> Point;

struct TreeSearchParams {
  float field_of_view;
  float distance;
  int max_search_iterations;
  int max_waypoints_per_path;
  float waypoint_field_of_view;
  float waypoint_distance;
  int path_queue_limit;
  float triangulation_radius;
  float triangulation_min_cone_dist;
  float triangulation_min_waypoint_dist;
};

class TreeSearch {
public:
  std::vector<Cone> cones;
  std::vector<Point> waypoints;
  TreeSearchParams params;
  CostParams cost_params;

  TreeSearch(std::vector<Cone> cones, TreeSearchParams params, CostParams cost_params);

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
