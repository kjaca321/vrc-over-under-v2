#include "main.h"

void initialize() {
  hang1.set(1);
  hang2.set(1);
  robot.setup();
  // master.rumble("-");
  robot.set_controller_tuning("arcade", 1, "exponential", 0.02, 10, 0.95);
  pros::lcd::initialize();
  master.rumble("-");
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  Trajectory1D::set_constraints(75, 180, 5);
  AngularTrajectory::set_constraints(12, 30, .1);
  run_auton_sequence();

  robot.turn_pt(Angle(135, Unit::DEGREES));
  robot.stop();

  // sys_task::hang_req = 0;
  // sys_task::intake_req = 1;
  // Trajectory1D::set_constraints(75, 180, 15);
  // robot.straight(12);
  // Trajectory1D::set_constraints(75, 180, 5);
  // robot.straight(-34.5);
  // robot.stop();
  // sys_task::intake_req = 0;
  // robot.turn_pt(Angle(-26, Unit::DEGREES));
  // robot.stop();
  // sys_task::left_wing_req = 1;
  // robot.straight(-24);
  // sys_task::left_wing_req = 0;
  // robot.turn_pt(Angle(-110, Unit::DEGREES));
  // robot.stop();
  // sys_task::right_wing_req = 1;
  // Trajectory1D::set_constraints(75, 180, 50);
  // robot.straight(-20);
  // robot.stop();
  // sys_task::right_wing_req = 0;
  kill_auton_sequence();
}

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