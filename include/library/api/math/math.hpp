/**
 * \file math.hpp
 *
 * Static math class, used for various mathematical functions.
 *
 * @author 3135B
 */

#pragma once
#include "../../util.hpp"

namespace lib::math {

class Math {
public:
  /**
   * Rounds the given number to the given precision.
   * @param num : number to round
   * @param prec : number of decimal places to round to
   * @return the rounded number
   */
  static float roundoff(float num, unsigned char prec);

  /**
   * Returns the sign (-1, 0, or 1) of a given number.
   * @param num : number that is evaluated
   * @return the sign of a number
   */
  static signed char sgn(float num);

  /**
   * Retrieves the minimum error angle between a target and actual angle.
   * @param target : target angle to reach
   * @param current : current robot angle
   * @return minimum turning angle
   */
  static float find_min_angle(float target, float current);

  /**
   * Retrieves the minimum error angle between an original error angle.
   * @param turn_angle : original error angle
   * @return minimum turning angle
   */
  static float find_min_angle(float turn_angle);
};

}; // namespace lib::math