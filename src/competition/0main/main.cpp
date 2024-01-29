#include "main.h"

void initialize() {
  hang1.set(1);
  hang2.set(1);
  robot.set_controller_tuning("arcade", 1, "exponential", 0.02, 10, 0.95);
  pros::lcd::initialize();
  auton::run_selection();
  master.rumble("-");
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  run_master_auton();
}

void opcontrol() {
  run_driver_sequence();
}