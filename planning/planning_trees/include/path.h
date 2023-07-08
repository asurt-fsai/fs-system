#pragma once

#include "cone.h"
#include "waypoint.h"
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <tuple>
#include <vector>

struct CostParams {
  float track_width;
  float track_width_weight;
  float color_weight;
  float angle_weight;
  float error_no_detect;
};

class Path {
public:
  // List of waypoints on this path
  std::vector<Waypoint> waypoints;

  // For each two waypoints in order [(w0, w1), (w1, w2), ...], this is a vector
  // that point to the left of them
  std::vector<std::array<float, 2>> left_unit_normals;

  // The set of cones that are currently detected
  std::vector<Cone> cones;

  std::vector<float> left_dists, right_dists;

  int undet_left = 0, undet_right = 0;

  CostParams cost_params;

  Path() {}
  Path(std::vector<Cone> cones, CostParams cost_params) {
    waypoints.emplace_back(0.0, 0.0);
    this->cones = cones;
    this->cost_params = cost_params;
  }

  /**
   * @brief Add a waypoint to the path and recomputes some parts of the costs if
   * needed
   *
   * @param x the x coordinate of the waypoint
   * @param y the y coordinate of the waypoint
   *
   * @return void
   */
  void addWaypoint(float x, float y);

  float getNearestConeToLine(std::vector<Cone> cones,
                             std::array<float, 2> left_unit_normal,
                             float intercept, Waypoint &new_waypoint,
                             Waypoint &prev_waypoint);

  std::vector<int> getIndices();

  float getColorCost();

  float getAngleCost();

  float getPathWidthCost();

  Waypoint &get_last_waypoint() { return waypoints.back(); }

  float getCost();

  Path createCopy();

  bool hasWaypoint(Waypoint waypoint);

  Path &operator=(const Path &rhs);

  bool operator==(const Path &other) const;
};
