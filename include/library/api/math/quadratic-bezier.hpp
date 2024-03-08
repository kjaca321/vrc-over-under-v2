/**
 * \file quadratic-bezier.hpp
 *
 * A QuadraticBezier generates a quadratic bezier curve from guiding points as
 * parameters.
 *
 * @author 3135B
 */

#pragma once
#include "vector.hpp"

namespace lib::math {

class QuadraticBezier {

public:
  QuadraticBezier(Vector end, float _lead);
  float arc_length;
  float lead;
  Vector target;
  Vector get(float length);

private:
  Vector get_raw(float t);
  float integral(float t);
};

} // namespace lib::math