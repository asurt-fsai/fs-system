#include "../include/path.h"

void Path::addWaypoint(float x, float y) {
  if (waypoints.size() == 0) {
    throw std::invalid_argument("Path must have at least one waypoint");
  }

  waypoints.emplace_back(x, y);
  Waypoint *new_waypoint = &waypoints[waypoints.size() - 1];
  Waypoint *prev_waypoint = &waypoints[waypoints.size() - 2];

  // Update headings of the waypoints
  new_waypoint->updateWaypointPrev(*prev_waypoint);
  prev_waypoint->updateWaypointNext(*new_waypoint);

  // Add the new waypoint to the path

  // Add the unit vector pointing to the left of the two waypoints
  std::array<float, 2> left_unit_normal =
      prev_waypoint->getLeftUnitNormal(*new_waypoint);
  left_unit_normals.push_back(left_unit_normal);

  // Cones on either side of the line connecting the two waypoints
  std::vector<Cone> left_cones;
  std::vector<Cone> right_cones;

  for (Cone cone : cones) {
    std::array<float, 2> diff = prev_waypoint->getVecTo({cone.x, cone.y});
    float dot_product =
        left_unit_normal[0] * diff[0] + left_unit_normal[1] * diff[1];
    if (dot_product < 0) { // Cone on the right side
      right_cones.push_back(cone);
    } else { // Cone on the left side
      left_cones.push_back(cone);
    }
  }

  // Get the distance to the nearest cone on either side
  float intercept = prev_waypoint->getIntercept(*new_waypoint);
  float nearest_left_cone_dist = Path::getNearestConeToLine(
      left_cones, left_unit_normal, intercept, *prev_waypoint, *new_waypoint);
  float nearest_right_cone_dist = Path::getNearestConeToLine(
      right_cones, left_unit_normal, intercept, *prev_waypoint, *new_waypoint);

  left_dists.push_back(nearest_left_cone_dist);
  right_dists.push_back(nearest_right_cone_dist);
}

float Path::getNearestConeToLine(std::vector<Cone> cones,
                                 std::array<float, 2> left_unit_normal,
                                 float intercept, Waypoint &prev_waypoint,
                                 Waypoint &new_waypoint) {
  if (cones.size() == 0) {
    return ERROR_NO_DETECT;
  }
  std::array<float, 4> bounds = new_waypoint.getBounds(prev_waypoint);

  float left_bound = std::get<0>(bounds);
  float right_bound = std::get<1>(bounds);
  float bottom_bound = std::get<2>(bounds);
  float top_bound = std::get<3>(bounds);

  std::vector<Cone> projected_cones;
  for (Cone cone : cones) {
    // Project the cone onto the line connecting the two waypoints
    float distance_to_line =
        cone.x * left_unit_normal[0] + cone.y * left_unit_normal[1] - intercept;
    float projected_x = cone.x - distance_to_line * left_unit_normal[0];
    float projected_y = cone.y - distance_to_line * left_unit_normal[1];

    if (left_bound < projected_x && projected_x < right_bound &&
        bottom_bound < projected_y && projected_y < top_bound) {
      projected_cones.push_back(Cone(projected_x, projected_y, 0));
    }
  }

  if (projected_cones.size() > 0) {
    return prev_waypoint.getDistNearestCone(projected_cones);
  } else {
    return prev_waypoint.getDistNearestCone(cones);
  }
}

std::vector<int> Path::getIndices() {
  std::vector<int> is_cone_right(cones.size(), 1); // Filled with ones

  float prev_heading = 0;
  for (int i = 0; i < waypoints.size() - 1; i++) {
    Waypoint *waypoint = &waypoints[i];
    std::array<float, 2> left_unit_normal = left_unit_normals[i];

    float heading = atan2(left_unit_normal[1], left_unit_normal[0]);
    float diff = heading - prev_heading;
    bool is_turning_left =
        diff >
        0; // Heading is increasing (turning left), TODO: fix pi to -pi bug
    if (i == 0)
      is_turning_left = false;

    prev_heading = heading;
    std::vector<int> is_inside(cones.size(), 0); // Filled with zeros
    for (int j = 0; j < cones.size(); j++) {
      Cone cone = cones[j];
      std::array<float, 2> vecToCone = waypoint->getVecTo({cone.x, cone.y});
      float dot_product = left_unit_normal[0] * vecToCone[0] +
                          left_unit_normal[1] * vecToCone[1];

      if (dot_product <= 0) { // Cone on the right side of the line between this
                              // waypoint and the next
        if (is_turning_left) { // Turning left
          is_cone_right[j] = 1;
        } else { // Turning right
          is_inside[j] = 1;
        }
      }
    }
    if (!is_turning_left) { // Turning right
      for (int j = 0; j < cones.size(); j++) {
        if (is_inside[j] == 0) {
          is_cone_right[j] = 0;
        }
      }
    }
  }
  return is_cone_right;
}

float Path::getColorCost() {
  std::vector<int> is_cone_right = Path::getIndices();

  float cost = 0;
  for (int i = 0; i < cones.size(); i++) {
    Cone cone = cones[i];
    cost += cone.getCost(is_cone_right[i] == 1);
  }
  return cost;
}

float Path::getAngleCost() {
  if (waypoints.size() == 0) {
    return 0;
  }

  float largest_angle_change = fabs(waypoints.front().heading);
  float current_angle_change;
  for (int i = 0; i < waypoints.size() - 1; i++) {
    current_angle_change =
        fabs(waypoints[i].heading - waypoints[i + 1].heading);
    largest_angle_change = std::max(largest_angle_change, current_angle_change);
  }
  return largest_angle_change;
}

float vectorMean(std::vector<float> vec, float trackwidth) {
  float sum = 0;
  for (float val : vec) {
    sum += (val - trackwidth / 2.0) * (val - trackwidth / 2.0);
  }
  return sum / (float)vec.size();
}

float Path::getPathWidthCost() {

  float path_width_cost = vectorMean(left_dists, TRACKWIDTH);
  path_width_cost += vectorMean(right_dists, TRACKWIDTH);
  path_width_cost = path_width_cost / 2.0;
  return path_width_cost;
}

float Path::getCost() {
  float cost = 0;

  cost += TRACKWIDTHWEIGHT * Path::getPathWidthCost();
  cost += COLORWEIGHT * Path::getColorCost();
  cost += ANGLEWEIGHT * Path::getAngleCost();

  return cost;
}

bool Path::hasWaypoint(Waypoint waypoint) {
  float epsilon = std::numeric_limits<float>::epsilon();
  for (int i = 0; i < waypoints.size(); i++) {
    if (fabs(waypoints[i].x - waypoint.x) <= epsilon &&
        fabs(waypoints[i].y - waypoint.y) <= epsilon) {
      return true;
    }
  }
  return false;
}

Path Path::createCopy() {
  Path new_path = Path();
  new_path.waypoints = std::vector<Waypoint>();
  for (int i = 0; i < waypoints.size(); i++) {
    new_path.waypoints.emplace_back(waypoints[i].x, waypoints[i].y);
    new_path.waypoints[i].manualCopy(waypoints[i]);
  }

  new_path.left_dists = left_dists;
  new_path.right_dists = right_dists;
  new_path.left_unit_normals = left_unit_normals;
  new_path.cones = cones;
  new_path.undet_left = undet_left;
  new_path.undet_right = undet_right;

  return new_path;
}
