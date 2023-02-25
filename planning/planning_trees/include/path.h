#pragma once

#include <cmath>
#include <tuple>
#include <array>
#include <vector>
#include <iostream>
#include <stdexcept>
#include "cone.h"
#include "waypoint.h"

class Path {
public:
  // List of waypoints on this path
  std::vector<Waypoint> waypoints;

  // For each two waypoints in order [(w0, w1), (w1, w2), ...], this is a vector that point to the left of them
  std::vector<std::array<float, 2>> left_unit_normals;

  // The set of cones that are currently detected
  std::vector<Cone> cones;

  std::vector<float> left_dists, right_dists;

  int undet_left = 0, undet_right = 0;

  float ERROR_NO_DETECT = 0;
  float TRACKWIDTH = 4;
  float TRACKWIDTHWEIGHT = 1;
  float COLORWEIGHT = 1;
  float ANGLEWEIGHT = 1;

  Path() {}
  Path(std::vector<Cone> cones) {
    waypoints.emplace_back(0.0, 0.0);
    this->cones = cones;
  }

  /**
   * @brief Add a waypoint to the path and recomputes some parts of the costs if needed
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

  bool hasWaypoint(Waypoint waypoint);
};
