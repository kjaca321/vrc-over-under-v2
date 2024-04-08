/**
 * \file cubic-bezier.hpp
 *
 * A CubicBezier generates a cubic bezier curve from guiding points as
 * parameters.
 *
 * @author 3135B
 */

#pragma once
#include "vector.hpp"

namespace lib::math {

class CubicBezier {

public:
  CubicBezier(Vector _guide, Vector end, float _lead);
  float arc_length;
  float lead;
  Vector target;
  Vector guide;
  Vector get(float length);
  Vector get_raw(float t);
  float get_curvature(float t);
};

} // namespace lib::math