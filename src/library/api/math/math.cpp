/**
 * \file math.hpp
 *
 * Static math class, used for various mathematical functions.
 *
 * @author 3135B
 */

#include "library/api/math/math.hpp"

namespace lib::math {

float Math::roundoff(float num, unsigned char prec) {
  float pow_10 = pow(10.0f, (float)prec);
  return round(num * pow_10) / pow_10;
}

signed char Math::sgn(float num) {
  return (num > 0) ? 1 : ((num < 0) ? -1 : 0);
}

float Math::find_min_angle(float target, float current) {
  float turn_angle = target - current;
  if (turn_angle > M_PI || turn_angle < -M_PI)
    turn_angle = -Math::sgn(turn_angle) * (2 * M_PI - std::abs(turn_angle));
  return turn_angle;
}

float Math::find_min_angle(float turn_angle) {
  if (turn_angle > M_PI || turn_angle < -M_PI)
    turn_angle = -Math::sgn(turn_angle) * (2 * M_PI - std::abs(turn_angle));
  return turn_angle;
}

}; // namespace lib::math