#include "../include/waypoint.h"

Waypoint::Waypoint(float x, float y) : x(x), y(y), heading(0), tangent(0) {}

Waypoint::Waypoint(float x, float y, float heading)
    : x(x), y(y), heading(heading), tangent(heading) {}

float Waypoint::getDistanceTo(float x, float y) {
  return sqrt(pow(this->x - x, 2) + pow(this->y - y, 2));
}

float Waypoint::getDistanceTo(const Waypoint &other_waypoint) {
  return sqrt(pow(this->x - other_waypoint.x, 2) +
              pow(this->y - other_waypoint.y, 2));
}

void Waypoint::updateWaypointPrev(const Waypoint &prev) {
  prev_x = prev.x;
  prev_y = prev.y;

  // Update heading and tangent
  float dx = x - prev.x;
  float dy = y - prev.y;
  heading = atan2(dy, dx);
  tangent = heading;
}

void Waypoint::updateWaypointNext(const Waypoint &next) {
  float dx = next.x - prev_x;
  float dy = next.y - prev_y;
  tangent = atan2(dy, dx);
}

std::array<float, 2>
Waypoint::getLeftUnitNormal(const Waypoint &next_waypoint) {
  float heading = atan2(next_waypoint.y - y, next_waypoint.x - x);

  std::array<float, 2> left_unit_normal{-1 * (float)sin(heading),
                                        (float)cos(heading)};

  return left_unit_normal;
}

float Waypoint::getIntercept(const Waypoint &other_waypoint) {
  std::array<float, 2> left_unit_normal =
      this->getLeftUnitNormal(other_waypoint);

  float intercept = left_unit_normal[0] * x + left_unit_normal[1] * y;

  return intercept;
}

std::array<float, 4> Waypoint::getBounds(const Waypoint &other_waypoint) {
  float left_bound = std::min(x, other_waypoint.x);
  float right_bound = std::max(x, other_waypoint.x);
  float bottom_bound = std::min(y, other_waypoint.y);
  float top_bound = std::max(y, other_waypoint.y);
  std::array<float, 4> bounds{left_bound, right_bound, bottom_bound, top_bound};

  return bounds;
}

float Waypoint::getDistNearestCone(std::vector<Cone> cones){
  float min_dist = std::numeric_limits<float>::infinity();
  for (Cone cone : cones){
    float dist = this->getDistanceTo(cone.x, cone.y);
    if (dist < min_dist){
      min_dist = dist;
    }
  }
  return min_dist;
}

std::array<float, 2> Waypoint::getVecTo(std::array<float, 2> other) {
  return {other[0] - x, other[1] - y};
}

bool Waypoint::operator==(const Waypoint &other) const {
  return abs(x - other.x) < 0.001 && abs(y - other.y) < 0.001;
}

void Waypoint::print() {
  std::cout << "Waypoint at (" << x << ", " << y << ") , Heading: " << heading
            << ", Tanget: " << tangent << std::endl;
}
