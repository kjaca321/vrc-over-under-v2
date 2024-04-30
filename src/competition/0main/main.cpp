#include "main.h"

void initialize() {
  robot.set_controller_tuning("arcade", 2.04, "exponential", 0.02, 10, 0.95);
  pros::lcd::initialize();
  auton::run_selection();
  master.rumble("-");
  pros::lcd::print(2, "READY");
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  run_auton_sequence();
  pros::lcd::print(3, "running");
  auton::run_auton();
  kill_auton_sequence();
}

void opcontrol() {
  if (auton::selected_auton == auton::num_autons) {
    run_auton_sequence();
    pros::lcd::print(3, "running");
    auton::skills();
    kill_auton_sequence();
  }
  run_driver_sequence();
}