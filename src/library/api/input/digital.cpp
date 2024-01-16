/**
 * \file digital.cpp
 *
 * Digital class, contains static methods to access the status of all
 * digital buttons.
 *
 * @author 3135B
 */
#include "library/api/input/digital.hpp"

namespace lib::input {

bool Digital::pressing(char b) {
  switch (b) {
  case Button::A:
    return master.get_digital(pros::E_CONTROLLER_DIGITAL_A);
  case Button::B:
    return master.get_digital(pros::E_CONTROLLER_DIGITAL_B);
  case Button::X:
    return master.get_digital(pros::E_CONTROLLER_DIGITAL_X);
  case Button::Y:
    return master.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
  case Button::UP:
    return master.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
  case Button::DOWN:
    return master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
  case Button::LEFT:
    return master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);
  case Button::RIGHT:
    return master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
  case Button::L1:
    return master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
  case Button::L2:
    return master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
  case Button::R1:
    return master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
  case Button::R2:
    return master.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
  default:
    return 0;
  }
}

bool Digital::pressed(char b) {
  switch (b) {
  case Button::A:
    return master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A);
  case Button::B:
    return master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B);
  case Button::X:
    return master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X);
  case Button::Y:
    return master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y);
  case Button::UP:
    return master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP);
  case Button::DOWN:
    return master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN);
  case Button::LEFT:
    return master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT);
  case Button::RIGHT:
    return master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT);
  case Button::L1:
    return master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1);
  case Button::L2:
    return master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2);
  case Button::R1:
    return master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1);
  case Button::R2:
    return master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2);
  default:
    return 0;
  }
}

void Digital::toggle_thread(void *param) {
  while (1) {
    std::uint32_t nw = pros::millis();
    if (!Button::a_pressed)
      if (Digital::pressed(Button::A))
        Button::a_pressed = 1;
    if (Button::a_pressed)
      if (Digital::pressed(Button::A))
        Button::a_pressed = 0;
    if (!Button::b_pressed)
      if (Digital::pressed(Button::B))
        Button::b_pressed = 1;
    if (Button::b_pressed)
      if (Digital::pressed(Button::B))
        Button::b_pressed = 0;
    if (!Button::x_pressed)
      if (Digital::pressed(Button::X))
        Button::x_pressed = 1;
    if (Button::x_pressed)
      if (Digital::pressed(Button::X))
        Button::x_pressed = 0;
    if (!Button::y_pressed)
      if (Digital::pressed(Button::Y))
        Button::y_pressed = 1;
    if (Button::y_pressed)
      if (Digital::pressed(Button::Y))
        Button::y_pressed = 0;
    if (!Button::up_pressed)
      if (Digital::pressed(Button::UP))
        Button::up_pressed = 1;
    if (Button::up_pressed)
      if (Digital::pressed(Button::UP))
        Button::up_pressed = 0;
    if (!Button::down_pressed)
      if (Digital::pressed(Button::DOWN))
        Button::down_pressed = 1;
    if (Button::down_pressed)
      if (Digital::pressed(Button::DOWN))
        Button::down_pressed = 0;
    if (!Button::left_pressed)
      if (Digital::pressed(Button::LEFT))
        Button::left_pressed = 1;
    if (Button::left_pressed)
      if (Digital::pressed(Button::LEFT))
        Button::left_pressed = 0;
    if (!Button::right_pressed)
      if (Digital::pressed(Button::RIGHT))
        Button::right_pressed = 1;
    if (Button::right_pressed)
      if (Digital::pressed(Button::RIGHT))
        Button::right_pressed = 0;
    if (!Button::l1_pressed)
      if (Digital::pressed(Button::L1))
        Button::l1_pressed = 1;
    if (Button::l1_pressed)
      if (Digital::pressed(Button::L1))
        Button::l1_pressed = 0;
    if (!Button::l2_pressed)
      if (Digital::pressed(Button::L2))
        Button::l2_pressed = 1;
    if (Button::l2_pressed)
      if (Digital::pressed(Button::L2))
        Button::l2_pressed = 0;
    if (!Button::r1_pressed)
      if (Digital::pressed(Button::R1))
        Button::r1_pressed = 1;
    if (Button::r1_pressed)
      if (Digital::pressed(Button::R1))
        Button::r1_pressed = 0;
    if (!Button::r2_pressed)
      if (Digital::pressed(Button::R2))
        Button::r2_pressed = 1;
    if (Button::r2_pressed)
      if (Digital::pressed(Button::R2))
        Button::r2_pressed = 0;
    pros::Task::delay_until(&nw, 5);
  }
}

}; // namespace lib::input