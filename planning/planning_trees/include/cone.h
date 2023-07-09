#pragma once

#include <cmath>
#include <iostream>
#include <stdexcept>

class Cone {
public:
  float x; // X location of the cone
  float y; // Y location of the cone

  // Probability for each color [prob_blue, prob_yellow, prob_orange]
  float color_probs[3] = {0.33, 0.33, 0.33};

  Cone(float cone_x, float cone_y, int cone_type);

  /**
   * @brief Get the distance between this cone and another point
   *
   * @param x The x coordinate of the other point
   * @param y The y coordinate of the other point
   *
   * @return float The distance between this cone and the other point
   */
  float getDistance(float x, float y);

  /**
   * @brief Compute the log likelihood costs given the position of the cone along the path
   *
   * @param is_right bool Whether the cone is on the right side of the path
   *
   * @return float The log likelihood cost
   */
  float getCost(bool is_right);

  /**
   * @brief print the cone's location, mainly for debugging
   *
   * @return void
   */
  void print();
};
