/**
 * \file point.cpp
 *
 * A Point inherits an x and y from a Vector, as well as all immutable methods
 * from the Vector class. In addition, a Point has a linear velocity, desired
 * heading, and curvature.
 *
 * @author 3135B
 */

#include "library/api/math/point.hpp"

namespace lib::math {

Point::Point() : Vector(), linear_velocity(0), curvature(0) {}

Point::Point(float inx, float iny)
    : Vector(inx, iny), linear_velocity(0), curvature(0) {}

Point::Point(float inx, float iny, math::Angle in_heading, float in_linear,
             float in_curv)
    : Vector(inx, iny), heading(in_heading), linear_velocity(in_linear),
      curvature(in_curv) {}

Point Point::closest_in(std::vector<Point> vec) {
  float closest_distance = this->distance(vec[0]);
  Point closest_pt = vec[0];
  for (Point x : vec)
    if (this->distance(x) < closest_distance) {
      closest_distance = this->distance(x);
      closest_pt = x;
    }
  return closest_pt;
}

std::string Point::to_string_full() {
  return to_string() + ",      Heading: " + heading.to_string() +
         ",      Linear: " + std::to_string(linear_velocity) +
         ",      Curvature: " + std::to_string(curvature);
}

bool Point::equals_pt(Point rhs) {
  return x == rhs.x && y == rhs.y && heading.equals(rhs.heading) &&
         linear_velocity == rhs.linear_velocity && curvature == rhs.curvature;
}

int Point::find_idx_in_pt(std::vector<Point> vec) {
  for (int i = 0; i < vec.size(); i++)
    if (this->equals(vec[i]))
      return i;
  return -1;
}

}; // namespace lib::math