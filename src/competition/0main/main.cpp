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
  pros::lcd::print(3, "running");
  // auton::run_auton();
  Trajectory2D::set_constraints(67, 100, 15, 11.0);
  // Trajectory2D traj(CubicBezier(Vector(0, 18), Vector(-14, 30), 6.0));

  // CubicBezier raw_path(Vector(20, 20), Vector(20, 20), 6.0);
  // std::vector<math::Vector> spline = {};

  // float density = 0.01;
  // for (float t = density; t <= 1; t += density) {
  //   math::Vector prev = raw_path.get_raw(t - density);
  //   math::Vector curr = raw_path.get_raw(t);
  //   float d = curr.distance(prev);
  //   spline.push_back(prev);
  // }
  // spline.push_back(raw_path.get_raw(1));

  // for (Vector i : spline) {
  //   std::cout << i.to_string() << std::endl;
  //   pros::delay(10);
  // }

  robot.follow_prim(CubicBezier(Vector(0, 18), Vector(-14, 30), 6.0), -1);
  // robot.stop();
  robot.move(-127);
  pros::delay(300);
  // robot.follow_prim(CubicBezier(Vector(2, 15), Vector(30, 20), 8.0), 1);
  robot.stop();
  pros::delay(2);
  kill_auton_sequence();
}

void opcontrol() {
  if (auton::selected_auton == auton::num_autons) {
    run_auton_sequence();
    pros::lcd::print(3, "running");
    auton::skills_start();
    kill_auton_sequence();
  }
  run_driver_sequence();
}