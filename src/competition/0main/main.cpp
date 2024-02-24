#include "main.h"

void initialize() {
  robot.set_controller_tuning("arcade", 1, "exponential", 0.02, 10, 0.95);
  pros::lcd::initialize();
  // auton::run_selection();
  robot.setup();
  master.rumble("-");
  pros::lcd::print(2, "READY");
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  run_auton_sequence();
  auton::safe_6b();
  kill_auton_sequence();
}

void opcontrol() {
  run_driver_sequence();
  while (1) {
    std::cout << "vel: " << (robot.get_left_vel() + robot.get_right_vel()) / 2
              << std::endl;
    pros::delay(25);
  }
}