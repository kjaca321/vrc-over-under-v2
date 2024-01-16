/**
 * \file unit.hpp
 *
 * Unit class, contains various static variables that represent units.
 *
 * @author 3135B
 */

#pragma once
#include "../../util.hpp"

namespace lib::math {

class Unit {
public:
  /**
   * An std::string static representation of the degree unit, used in the Angle
   * class.
   */
  static const std::string DEGREES;

  /**
   * An std::string static representation of the radian unit, used in the Angle
   * class.
   */
  static const std::string RADIANS;
};

}; // namespace lib::math