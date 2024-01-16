/**
 * \file globals.cpp
 *
 * Globals, contains various static variables that control aspects of
 * the robot.
 *
 * @author 3135B
 */

#include "library/api/input/globals.hpp"

namespace lib::input {

pros::Controller master = pros::Controller(pros::E_CONTROLLER_MASTER);

}; // namespace lib::input