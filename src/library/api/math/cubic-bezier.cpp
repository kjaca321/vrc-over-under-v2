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
  float x = 3 * t * t * (1 - t) * guide.x + t * t * t * target.x;
  float y = 3 * t * (1 - t) * (1 - t) * lead + 3 * t * t * (1 - t) * guide.y +
            t * t * t * target.y;
  return Vector(x, y);
}

Vector CubicBezier::get(float length) {
  float t = length / arc_length;
  return get_raw(t);
}

} // namespace lib::math