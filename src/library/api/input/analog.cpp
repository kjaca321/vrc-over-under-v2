/**
 * \file analog.cpp
 *
 * Analog class, contains static methods to access the joystick values.
 *
 * @author 3135B
 */
#include "library/api/input/analog.hpp"

namespace lib::input {

int Analog::get_left_x() {
  return master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
}

int Analog::get_left_y() {
  return master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
}

int Analog::get_right_x() {
  return master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
}

int Analog::get_right_y() {
  return master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
}

}; // namespace lib::input