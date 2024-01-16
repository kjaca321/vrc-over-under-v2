/**
 * \file chassis.hpp
 *
 * Chassis class, contains methods to group and control
 * the chassis motors and hardware.
 *
 * @author 3135B
 */

#pragma once
#include "../../util.hpp"
#include "../util/braketype.hpp"

namespace lib::control {

class Chassis {
public:
  /**
   * Constructs a Chassis based on a set of left and right ports.
   * @param left_ports : set of left ports
   * @param right_ports : set of right ports
   */
  Chassis(std::vector<int> left_ports, std::vector<int> right_ports);

  /**
   * Moves the chassis motors at the desired voltage.
   * @param voltage : voltage at which the chassis moves
   */
  void move(float voltage);

  /**
   * Moves the left side of the chassis motors at the desired voltage.
   * @param voltage : voltage at which the left side moves
   */
  void move_left(float voltage);

  /**
   * Moves the right side of the chassis motors at the desired voltage.
   * @param voltage : voltage at which the right side moves
   */
  void move_right(float voltage);

  /**
   * Moves the chassis motors at the desired voltage, with the right side
   * inversed.
   * @param voltage : voltage at which the chassis moves
   */
  void turn(float voltage);

  /**
   * Sets the brake type of the chassis motors to one of three (coast, brake,
   * hold).
   * @param brake : brake type of chassis
   */
  void set_brake(pros::motor_brake_mode_e brake);

  /**
   * Sets the brake type of the left chassis motors to one of three (coast,
   * brake, hold).
   * @param brake : brake type of chassis
   */
  void set_left_brake(pros::motor_brake_mode_e brake);

  /**
   * Sets the brake type of the right chassis motors to one of three (coast,
   * brake, hold).
   * @param brake : brake type of chassis
   */
  void set_right_brake(pros::motor_brake_mode_e brake);

  /**
   * Stops the chassis by setting the input voltage to 0.
   */
  void brake(void);

  /**
   * Full stopping motion used at the end of autonmous motions.
   */
  void stop(void);

protected:
  std::vector<pros::Motor *> left_motors;
  std::vector<pros::Motor *> right_motors;
};

}; // namespace lib::control