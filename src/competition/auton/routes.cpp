#include "main.h"

namespace auton {

float MAX_SPEED = 67;
float MAX_ACCEL = 180;

void safe_6b() {
  sys_task::hang_req = 0;
  sys_task::intake_req = 1;
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 15);
  robot.straight(12);
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 5);
  robot.straight(-34.5);
  robot.stop();
  robot.turn_pt(Angle(-26, Unit::DEGREES), 0);
  robot.stop();
  sys_task::left_wing_req = 1;
  sys_task::right_wing_req = 1;
  sys_task::intake_req = 0;
  robot.straight(-22.4);
  sys_task::left_wing_req = 0;
  robot.turn_pt(Angle(-81, Unit::DEGREES), 1);
  robot.stop();
  // sys_task::right_wing_req = 1;

  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 50);
  sys_task::right_wing_req = 0;
  robot.straight(-35);
  robot.stop();
  // Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 5);
  robot.straight(11);
  robot.turn_pt(Angle(81.5, Unit::DEGREES), 1);
  sys_task::intake_rev_req = 1;
  pros::delay(250);
  robot.straight(31);
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 5);
  robot.stop();
  pros::delay(200);
  robot.straight(-8);
  sys_task::intake_rev_req = 0;
  robot.turn_pt(Angle(28, Unit::DEGREES), 0);
  sys_task::intake_req = 1;
  robot.straight(68);
  robot.straight(-6);
  robot.stop();
  robot.turn_pt(Angle(140, Unit::DEGREES), 0);
  robot.stop();
  sys_task::intake_req = 0;
  sys_task::intake_speed = 70;
  sys_task::intake_rev_req = 1;
  pros::delay(500);
  sys_task::intake_rev_req = 0;
  sys_task::intake_speed = 127;
  sys_task::intake_req = 1;
  robot.turn_pt(Angle(65, Unit::DEGREES), 0);
  robot.straight(40);
  robot.stop();
  robot.straight(-2);
  robot.turn_pt(Angle(180, Unit::DEGREES), 0);
  robot.stop();
  sys_task::intake_req = 0;
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 50);
  sys_task::front_wings_req = 1;
  sys_task::intake_rev_req = 1;
  pros::delay(200);
  robot.straight(50);
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 5);
  sys_task::front_wings_req = 0;
  sys_task::intake_rev_req = 0;
  pros::delay(150);
  robot.straight(-10);
  robot.stop();
  // sys_task::right_wing_req = 0;
}

} // namespace auton