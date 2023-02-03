#include "../include/cone.h"

Cone::Cone(float cone_x, float cone_y, int cone_type) : x(cone_x), y(cone_y) {
  if (cone_type == 0) { // Blue Cone
    color_probs[0] = 0.8;
    color_probs[1] = 0.1;
    color_probs[2] = 0.1;
  } else if (cone_type == 1) { // Yellow Cone
    color_probs[0] = 0.1;
    color_probs[1] = 0.8;
    color_probs[2] = 0.1;
  } else if (cone_type == 2) { // Orange Cone
    color_probs[0] = 0.1;
    color_probs[1] = 0.1;
    color_probs[2] = 0.8;
  } else {
    throw std::invalid_argument("Invalid cone type: " +
                                std::to_string(cone_type));
  }
}

float Cone::getDistance(float x, float y) {
  return sqrt(pow(this->x - x, 2) + pow(this->y - y, 2));
}

float Cone::getCost(bool is_right) {
  float cost = 0.0;
  if (is_right) {
    //cost -= 0.5*log(1 - color_probs[0]);
    //cost += log(1 - color_probs[1]);
    cost -= log(color_probs[1]);
    //cost -= 0.5 * log(1 - color_probs[2]);
  }
  else
  {
    //cost += log(1 - color_probs[0]);
    cost -= log(color_probs[0]);
    //cost -= 0.5*log(1 - color_probs[1]);
    //cost -= 0.5*log(1 - color_probs[2]);
  }
  return cost;
}

void Cone::print() {
  std::cout << "Cone at (" << x << ", " << y << ")" << std::endl;
}
