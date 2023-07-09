#include "../include/tree_search.h"
#include "../include/path_queue.h"

TreeSearch::TreeSearch(std::vector<Cone> cones, TreeSearchParams params, CostParams cost_params) {
  this->params = params;
  this->cost_params = cost_params;
  this->cones = TreeSearch::filterLocal(cones, this->params.field_of_view,
                                        this->params.distance);
  this->waypoints = TreeSearch::triangulate(this->cones);
}

template <typename T>
std::vector<T> TreeSearch::filterLocal(std::vector<T> points,
                                       float field_of_view, float distance) {
  return TreeSearch::filterLocal(points, field_of_view, distance, 0, 0, 0);
}

std::vector<Cone> TreeSearch::filterLocal(std::vector<Cone> points,
                                          float field_of_view, float distance,
                                          float x, float y, float heading) {
  std::vector<Cone> filtered_points;

  for (Cone point : points) {
    float dx = point.x - x;
    float dy = point.y - y;
    float dist = sqrt(dx * dx + dy * dy);
    float angle = atan2(dy, dx) - heading;

    if (angle < -M_PI)
      angle += 2 * M_PI; // Normalize angle to [-pi, pi]
    if (angle > M_PI)
      angle -= 2 * M_PI; // Normalize angle to [-pi, pi]

    if (fabs(angle) <= field_of_view / 2 && dist <= distance) {
      filtered_points.push_back(point);
    }
  }
  return filtered_points;
}

std::vector<Point> TreeSearch::filterLocal(std::vector<Point> points,
                                           float field_of_view, float distance,
                                           float x, float y, float heading) {
  std::vector<Point> filtered_points;

  // std::cout << "filtering local" << x << "," << y << "," << heading << std::endl;
  for (int i = 0; i < points.size(); i++) {
    float dx = points[i][0] - x;
    float dy = points[i][1] - y;
    float dist = sqrt(dx * dx + dy * dy);
    float angle = atan2(dy, dx) - heading;
    // std::cout << "point: " << points[i][0] << "," << points[i][1] << std::endl;
    // std::cout << "angle: " << angle << ", dist: " << dist << std::endl;

    if (angle < -M_PI)
      angle += 2 * M_PI; // Normalize angle to [-pi, pi]
    if (angle > M_PI)
      angle -= 2 * M_PI; // Normalize angle to [-pi, pi]

    if (fabs(angle) <= field_of_view / 2 && dist <= distance) {
      filtered_points.push_back({points[i][0], points[i][1]});
    }
  }
  return filtered_points;
}

std::vector<Point> TreeSearch::triangulate(std::vector<Cone> cones) {
  std::vector<Point> waypoints;
  for (Cone cone : cones) {
    // Add 8 waypoints around the cone
    for (int i = 0; i < 8; i++) {
      float angle = i * M_PI / 4;
      float x = cone.x + this->params.triangulation_radius * cos(angle);
      float y = cone.y + this->params.triangulation_radius * sin(angle);

      // find the closest cone to the waypoint
      Waypoint waypoint(x, y);
      float closest_cone_dist = waypoint.getDistNearestCone(cones);

      // Find closest waypoint to this waypoint
      float closest_waypoint_dist = std::numeric_limits<float>::infinity();
      for (int i = 0; i < waypoints.size(); i++) {
        float dist = waypoint.getDistanceTo(waypoints[i][0], waypoints[i][1]);
        if (dist < closest_waypoint_dist) {
          closest_waypoint_dist = dist;
        }
      }
      if (closest_cone_dist > this->params.triangulation_min_cone_dist &&
          closest_waypoint_dist >
              this->params.triangulation_min_waypoint_dist) {
        waypoints.push_back({x, y});
      }
    }
  }
  return waypoints;
}

bool has_path(std::vector<Path> all_paths, Path path) {
  for (Path p : all_paths) {
    if (p == path) {
      return true;
    }
  }
  return false;
}
std::tuple<Path, std::vector<Path>> TreeSearch::getPath() {
  PathQueue path_queue = PathQueue(params.path_queue_limit);
  Path start_path = Path(this->cones, this->cost_params);
  std::vector<Path> all_paths;
  all_paths.push_back(start_path);
  path_queue.addNewPath(start_path, std::numeric_limits<float>::infinity());

  for (int i = 0; i < params.max_search_iterations; i++) {
    bool no_new_paths = true;
    int size = path_queue.paths.size();
    for (int j = 0; j < size; j++) {
      Path curr_path = path_queue.paths.front();
      float curr_cost = path_queue.costs.front();
      path_queue.paths.erase(path_queue.paths.begin());
      path_queue.costs.erase(path_queue.costs.begin());

      if (curr_path.waypoints.size() > params.max_waypoints_per_path) {
        path_queue.addNewPath(curr_path, curr_cost);
        continue;
      }
      Waypoint last_waypoint = curr_path.waypoints.back();
      // std::cout << " Filtering waypoints " << std::endl;

      std::vector<Point> possible_next_waypoints =
          filterLocal(this->waypoints, params.waypoint_field_of_view,
                      params.waypoint_distance, last_waypoint.x,
                      last_waypoint.y, last_waypoint.heading);
      // std::cout << "Possible next waypoints: " << possible_next_waypoints.size()
      //           << " all waypoints: " << this->waypoints.size() << std::endl;
      for (int k = 0; k < possible_next_waypoints.size(); k++) {
        if (curr_path.hasWaypoint(Waypoint(possible_next_waypoints[k][0],
                                           possible_next_waypoints[k][1]))) {
          continue;
        }
        Path new_path = curr_path.createCopy();
        new_path.addWaypoint(possible_next_waypoints[k][0], possible_next_waypoints[k][1]);
        if (has_path(all_paths, new_path)) {
          continue;
        }
        all_paths.push_back(new_path);
        float new_cost = new_path.getCost();
        bool added_path = path_queue.addNewPath(new_path, new_cost);
        if (added_path) {
          no_new_paths = false;
        }
      }
      path_queue.addNewPath(curr_path, curr_cost);
    }
    if (no_new_paths) {
      break;
    }
  }

  int best_path_index = path_queue.getBestPathIndex();
  Path to_return = path_queue.paths[best_path_index].createCopy();
  return std::tuple<Path, std::vector<Path>>(to_return, all_paths);
}
