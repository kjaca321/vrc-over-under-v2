#include "main.h"

void initialize() {
  robot.setup();
  master.rumble("-");
  robot.set_controller_tuning("arcade", 0.9, "exponential", 0.015, 10, 0.95);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
  // Trajectory1D::set_constraints(75, 230, 5);
  // Trajectory2D::set_constraints(75, 230, 5, 10);
  // AngularTrajectory::set_constraints(12, 35, .1);
  // robot.straight(50);
  // robot.stop();
  // robot.turn_pt(Angle(90, Unit::DEGREES));
  // robot.stop();
  // AngularTrajectory test(math::Angle(M_PI_2, Unit::RADIANS));
  // for (math::Pose1D pose : test.get()) {
  //   float des_dist = pose.pos;
  //   float omega = pose.vel;
  //   float alpha = pose.acc;
  //   float track = 10;
  //   robot.move_left(omega * track);
  //   robot.move_right(-omega * track);
  //   pros::delay(10);
  // }
  run_driver_sequence();
  // while (1) {
  //   float vel = robot.get_angular_vel();
  //   float acc = robot.get_angular_acc();
  //   std::cout << acc << std::endl;
  //   pros::delay(20);
  // }
}