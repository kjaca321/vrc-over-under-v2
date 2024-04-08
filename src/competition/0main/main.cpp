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
  auton::safe_6b();
  Trajectory2D::set_constraints(70, 130, 15, 11.0);
  // Trajectory2D traj(CubicBezier(Vector(0, 18), Vector(-14, 30), 6.0));
  robot.set_brake(BrakeType::BRAKE);
  robot.follow_prim(CubicBezier(Vector(30, 30), Vector(30, 30), 15.0), 1);
  robot.brake();
  pros::delay(100000);
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
  kill_auton_sequence();
}

void opcontrol() {
  // if (auton::selected_auton == auton::num_autons) {
  run_auton_sequence();
  //   pros::lcd::print(3, "running");
  //   auton::skills_start();
  //   kill_auton_sequence();
  // }
  // run_driver_sequence();

  // for (int i = 0; i < 128; i += 5) {
  //   robot.move(i);
  //   float vel = (robot.get_left_vel() + robot.get_right_vel()) / 2;
  //   std::cout << Vector(vel, i).to_string() << std::endl;
  //   pros::delay(50);
  // }
  // robot.move(0);
  Trajectory1D::set_constraints(70, 130, 1);
  Trajectory2D::set_constraints(70, 130, 5, 11.0);
  // robot.set_brake(BrakeType::HOLD);
  // robot.straight(24);
  // robot.move(0);
  // robot.move(-127);
  // pros::delay(100);
  // robot.brake();
  // pros::delay(2000);
  // robot.brake();

  robot.follow_prim(CubicBezier(Vector(25, 25), Vector(38, 50), 25.0), 1);

  // Trajectory2D a(CubicBezier(Vector(30, 30), Vector(30, 30), 15.0));
  robot.brake();

  // robot.follow_prim(CubicBezier(Vector(30, 30), Vector(30, 30), 15.0), 1);
  robot.move(0);
  // robot.move(-127);
  // pros::delay(30);
  robot.brake();

  while (1)
    pros::delay(2);

  kill_auton_sequence();
  pros::delay(2);
}