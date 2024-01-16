/**
 * \file button.cpp
 *
 * Button class, contains variables that represent all the digital buttons
 * on the controller and their status.
 *
 * @author 3135B
 */
#include "library/api/input/button.hpp"

namespace lib::input {

bool Button::a_pressed = 0;
bool Button::b_pressed = 0;
bool Button::x_pressed = 0;
bool Button::y_pressed = 0;
bool Button::up_pressed = 0;
bool Button::down_pressed = 0;
bool Button::left_pressed = 0;
bool Button::right_pressed = 0;
bool Button::l1_pressed = 0;
bool Button::l2_pressed = 0;
bool Button::r1_pressed = 0;
bool Button::r2_pressed = 0;

}; // namespace lib::input