/**
 * \file braketype.cpp
 *
 * BrakeType class, contains static variables that refer to pros brake modes.
 *
 * @author 3135B
 */
#include "library/api/util/braketype.hpp"

namespace lib::utility {

const pros::motor_brake_mode_e BrakeType::COAST = pros::E_MOTOR_BRAKE_COAST;
const pros::motor_brake_mode_e BrakeType::BRAKE = pros::E_MOTOR_BRAKE_BRAKE;
const pros::motor_brake_mode_e BrakeType::HOLD = pros::E_MOTOR_BRAKE_HOLD;

}; // namespace lib::utility