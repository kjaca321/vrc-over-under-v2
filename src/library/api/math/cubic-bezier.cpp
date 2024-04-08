/**
 * \file cubic-bezier.cpp
 *
 * A CubicBezier generates a cubic bezier curve from guiding points as
 * parameters.
 *
 * @author 3135B
 */

#include "library/api/math/cubic-bezier.hpp"

namespace lib::math {

CubicBezier::CubicBezier(Vector _guide, Vector end, float _lead) {
  target = end;
  lead = _lead;
  guide = _guide;
  float density = 0.004;
  arc_length = 0;
  for (float t = density; t <= 1; t += density) {
    Vector prev = get_raw(t - density);
    Vector curr = get_raw(t);
    float distance = curr.distance(prev);
    arc_length += distance;
  }
}

Vector CubicBezier::get_raw(float t) {
  /* parametric equations for a cubic bezier curve, returns an (x,y) position
   * vector from a time t on (0,1), based on a target point, guiding point, and
   * leading distance */
  float x = 3 * t * t * (1 - t) * guide.x + t * t * t * target.x;
  float y = 3 * t * (1 - t) * (1 - t) * lead + 3 * t * t * (1 - t) * guide.y +
            t * t * t * target.y;
  return Vector(x, y);
}

float CubicBezier::get_curvature(float t) {
  float x1 = 3 * t * (t * target.x + (2 - 3 * t) * guide.x);
  float y1 = 3 * (t * t * target.y + (2 - 3 * t) * t * guide.y +
                  lead * (3 * t * t - 4 * t + 1));
  float x2 = 6 * (t * target.x + (1 - 3 * t) * guide.x);
  float y2 = 6 * (t * target.y + (1 - 3 * t) * guide.y + lead * (3 * t - 2));
  float cross = fabs((x1 * y2) - (y1 * x2));
  float cube = pow(sqrt(x1 * x1 + y1 * y1), 3);
  return cross / cube;
}

Vector CubicBezier::get(float length) {
  float t = length / arc_length;
  return get_raw(t);
}

} // namespace lib::math