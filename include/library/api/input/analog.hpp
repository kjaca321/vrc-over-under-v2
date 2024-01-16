/**
 * \file analog.hpp
 *
 * Analog class, contains static methods to access the joystick values.
 *
 * @author 3135B
 */

#pragma once
#include "../../util.hpp"
#include "globals.hpp"

namespace lib::input {

class Analog {
public:
  /**
   * Retrieves the value of the left joystick, horizontally.
   * @return left horizontal joystick movement
   */
  static int get_left_x(void);

  /**
   * Retrieves the value of the left joystick, vertically.
   * @return left vertical joystick movement
   */
  static int get_left_y(void);

  /**
   * Retrieves the value of the right joystick, horizontally.
   * @return right horizontal joystick movement
   */
  static int get_right_x(void);

  /**
   * Retrieves the value of the right joystick, vertically.
   * @return right vertical joystick movement
   */
  static int get_right_y(void);
};

}; // namespace lib::input