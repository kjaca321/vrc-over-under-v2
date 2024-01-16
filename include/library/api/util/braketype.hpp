/**
 * \file braketype.hpp
 *
 * BrakeType class, contains variables that represent different PROS brake
 * modes.
 *
 * @author 3135B
 */

#pragma once
#include "../../util.hpp"

namespace lib::utility {

class BrakeType {
public:
  /**
   * Represents the coast brake mode.
   */
  static const pros::motor_brake_mode_e COAST;

  /**
   * Represents the brake brake mode.
   */
  static const pros::motor_brake_mode_e BRAKE;

  /**
   * Represents the hold brake mode.
   */
  static const pros::motor_brake_mode_e HOLD;
};

} // namespace lib::utility