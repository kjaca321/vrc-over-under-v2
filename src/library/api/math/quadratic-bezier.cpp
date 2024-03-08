/**
 * \file quadratic-bezier.cpp
 *
 * A QuadraticBezier generates a quadratic bezier curve from guiding points as
 * parameters.
 *
 * @author 3135B
 */

#include "library/api/math/quadratic-bezier.hpp"

namespace lib::math {

QuadraticBezier::QuadraticBezier(Vector end, float _lead) {
  target = end;
  lead = _lead;
  float density = 0.004;
  arc_length = 0;
  for (float t = density; t <= 1; t += density) {
    Vector prev = get_raw(t - density);
    Vector curr = get_raw(t);
    float distance = curr.distance(prev);
    arc_length += distance;
  }
}

Vector QuadraticBezier::get_raw(float t) {
  float x = t * t * target.x;
  float y = 2 * t * (1 - t) * lead + t * t * target.y;
  return Vector(x, y);
}

Vector QuadraticBezier::get(float length) {
  float t = length / arc_length;
  return get_raw(t);
}

float QuadraticBezier::integral(float t) {
  float c = lead, a = target.x, b = target.y;
  float res =
      1 / (a * a + pow(b - 2 * c, 2)) *
          sqrt(t * t * (a * a + b * b) + 2 * b * c * t * (1 - 2 * t) +
               c * c * pow(1 - 2 * t, 2)) *
          (a * a * t + b * b * t + b * (c - 4 * c * t) + c * c * (4 * t - 2)) +
      1 / (pow(a * a + pow(b - 2 * c, 2), 1.5)) * a * a * c * c *
          log(sqrt(a * a + pow(b - 2 * c, 2)) *
                  sqrt(t * t * (a * a + b * b) + 2 * b * c * t * (1 - 2 * t) +
                       c * c * pow(1 - 2 * t, 2)) +
              a * a * t + b * b * t + b * (c - 4 * c * t) + 4 * c * c * t -
              2 * c * c);
  return res;
}

} // namespace lib::math