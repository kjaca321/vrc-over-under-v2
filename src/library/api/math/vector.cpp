/**
 * \file vector.cpp
 *
 * A Vector has an x and a y. The class contains several immutable methods to
 * retrieve aspects of the Vector object.
 *
 * @author 3135B
 */

#include "library/api/math/vector.hpp"

namespace lib::math {

Vector::Vector() : x(0), y(0) {}

Vector::Vector(float inx, float iny) : x(inx), y(iny) {}

Vector Vector::operator+(const Vector &rhs) {
  return Vector(x + rhs.x, y + rhs.y);
}

Vector Vector::operator-(const Vector &rhs) {
  return Vector(x - rhs.x, y - rhs.y);
}

float Vector::operator*(const Vector &rhs) { return x * rhs.x + y * rhs.y; }

Vector Vector::operator*(const float num) { return Vector(x * num, y * num); }

float Vector::magnitude() { return sqrt(x * x + y * y); }

Vector Vector::normalize() {
  Vector z;
  float m = this->magnitude();
  if (m == 0)
    return z;
  Vector res(x / m, y / m);
  return res;
}

std::string Vector::to_string() {
  return "(" + std::to_string(x) + ", " + std::to_string(y) + ")";
}

bool Vector::equals(Vector rhs) { return x == rhs.x && y == rhs.y; }

float Vector::distance(Vector rhs) {
  return sqrt(pow(x - rhs.x, 2) + pow(y - rhs.y, 2));
}

bool Vector::is_in(std::vector<Vector> vec) {
  for (Vector x : vec)
    if (this->equals(x))
      return true;
  return false;
}

Vector Vector::closest_in(std::vector<Vector> vec) {
  float closest_distance = this->distance(vec[0]);
  Vector closest_pt = vec[0];
  for (Vector x : vec)
    if (this->distance(x) < closest_distance) {
      closest_distance = this->distance(x);
      closest_pt = x;
    }
  return closest_pt;
}

int Vector::find_idx_in(std::vector<Vector> vec) {
  for (int i = 0; i < vec.size(); i++)
    if (this->equals(vec[i]))
      return i;
  return -1;
}

}; // namespace lib::math