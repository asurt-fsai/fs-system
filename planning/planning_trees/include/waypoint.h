#pragma once

#include "cone.h"
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <tuple>
#include <vector>

class Waypoint {
public:
  float x;
  float y;

  // Position of the previous waypoint
  float prev_x;
  float prev_y;

  // Heading ( in radians) of the vector between the previous waypoint and this
  // waypoint
  float heading;

  // Tangent (in radians) to the path between the previous, current, and next
  // waypoints Effectively, this is the heading between the previous waypoint
  // and the next waypoint If the next waypoint is not known, this is the same
  // as the heading
  float tangent;

  // Initializes heading and tangent to 0
  Waypoint(float x, float y);

  // Initializes heading and tangent to the given float heading
  Waypoint(float x, float y, float heading);

  // Delete copy constructor to prevent accidental copying
  Waypoint(const Waypoint &) = delete;
  Waypoint(Waypoint &&) = default;

  /**
   * @brief Get the distance between this waypoint and another point
   *
   * @param x The x coordinate of the other point
   * @param y The y coordinate of the other point
   *
   * @return float The distance between this waypoint and the other point
   */
  float getDistanceTo(float x, float y);

  /**
   * @brief Get the distance between this waypoint and another waypoint
   *
   * @param other_waypoint The other waypoint
   *
   * @return float The distance between this waypoint and the other waypoint
   */
  float getDistanceTo(const Waypoint &other_waypoint);

  /**
   *
   * @brief Update the "heading" and "tangent" of the waypoint using the
   * previous waypoint Both variables are updated to the same value
   *
   * @param prev The previous waypoint
   */
  void updateWaypointPrev(const Waypoint &prev);

  /**
   *
   * @brief Update the "heading" variable of the waypoint as the average of the
   * heading between 1) The previous waypoint and current waypoint 2) The
   * current waypoint and the next waypoint The "heading" is updated to the
   * average between these two headings
   *
   * @param next The next waypoint
   */
  void updateWaypointNext(const Waypoint &next);

  /**
   *
   * @brief Get the left unit normal of the line between this waypoint and the
   * next waypoint
   *
   * @param next_waypoint The other waypoint
   *
   * @return std::array<float, 2> The left unit normal of the line between this
   * waypoint and the other waypoint
   */
  std::array<float, 2> getLeftUnitNormal(const Waypoint &next_waypoint);

  /**
   *
   * @brief Compute the y intercept of the line between this waypoint and
   * another waypoint
   *
   * @param other_waypoint The other waypoint
   *
   * @return float The y intercept of the line between this waypoint and the
   * other waypoint
   */
  float getIntercept(const Waypoint &other_waypoint);

  /**
   *
   * @brief Get the bounds of the square Containing this waypoint and another
   * waypoint
   *
   * @param other_waypoint The other waypoint
   *
   * @return std::array<float, 4> The bounds of the line between this waypoint
   * and the other waypoint The bounds are in the following order: min_x, max_x,
   * min_y, max_y
   */
  std::array<float, 4> getBounds(const Waypoint &other_waypoint);

  /**
   *
   * @brief Get the vector from this waypoint to another position
   *
   * @param other Another position
   *
   * @return std::array<float, 2> The vector from this waypoint to the other
   * waypoint
   */
  std::array<float, 2> getVecTo(std::array<float, 2> other);

  /**
   * @brief Get the distance to the nearest cone to this waypoint
   *
   * @param cones A vector of cones, where each cone is represented by a pair of
   * floats (x,y)
   *
   * @return float The distance to the nearest cone
   */
  float getDistNearestCone(std::vector<Cone> cones);

  /**
   * @brief Check if this waypoint is equal to another waypoint
   *
   * @param other The other waypoint
   *
   * @return true if the waypoints' x and y positions are within 0.001 of each
   * other
   */
  bool operator==(const Waypoint &other) const;

  /**
   * @brief Copy all values from another waypoint to this waypoint
   */
  void manualCopy(const Waypoint &other);

  /**
   * @brief Prints the current waypoint's position (x,y), heading, and tanget
   */
  void print();
};
