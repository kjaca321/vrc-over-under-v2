/**
 * \file chassis->cpp
 *
 * Chassis class, contains methods to group and control
 * the chassis motors and hardware.
 *
 * @author 3135B
 */
#include "library/api/control/chassis.hpp"

namespace lib::control {

Chassis::Chassis(std::vector<int> left_ports, std::vector<int> right_ports) {
  left_motors = {};
  right_motors = {};
  for (int i : left_ports)
    left_motors.push_back(new pros::Motor(i, pros::E_MOTOR_GEARSET_06,
                                          pros::E_MOTOR_ENCODER_DEGREES));
  for (int i : right_ports)
    right_motors.push_back(new pros::Motor(i, pros::E_MOTOR_GEARSET_06,
                                           pros::E_MOTOR_ENCODER_DEGREES));
}

void Chassis::move(float voltage) {
  for (pros::Motor *i : left_motors)
    i->move(voltage);
  for (pros::Motor *i : right_motors)
    i->move(voltage);
}

void Chassis::move_left(float voltage) {
  for (pros::Motor *i : left_motors)
    i->move(voltage);
}

void Chassis::move_right(float voltage) {
  for (pros::Motor *i : right_motors)
    i->move(voltage);
}

void Chassis::turn(float voltage) {
  for (pros::Motor *i : left_motors)
    i->move(voltage);
  for (pros::Motor *i : right_motors)
    i->move(-voltage);
}

void Chassis::set_brake(pros::motor_brake_mode_e brake) {
  for (pros::Motor *i : left_motors)
    i->set_brake_mode(brake);
  for (pros::Motor *i : right_motors)
    i->set_brake_mode(brake);
}

void Chassis::set_left_brake(pros::motor_brake_mode_e brake) {
  for (pros::Motor *i : left_motors)
    i->set_brake_mode(brake);
}

void Chassis::set_right_brake(pros::motor_brake_mode_e brake) {
  for (pros::Motor *i : right_motors)
    i->set_brake_mode(brake);
}

void Chassis::brake() {
  for (pros::Motor *i : left_motors)
    i->brake();
  for (pros::Motor *i : right_motors)
    i->brake();
}

void Chassis::stop() {
  move(0);
  set_brake(utility::BrakeType::BRAKE);
  brake();
  pros::delay(350);
  set_brake(utility::BrakeType::COAST);
  pros::delay(1);
}

}; // namespace lib::control