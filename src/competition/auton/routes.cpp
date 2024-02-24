#include "main.h"

namespace auton {

float MAX_SPEED = 67;
float MAX_ACCEL = 130;

void safe_close() {
  sys_task::intake_req = 1;
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 10);
  robot.turn_pt(Angle(45, Unit::DEGREES));
  robot.straight(-20);
  robot.turn_pt(Angle(90, Unit::DEGREES));
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 50);
  robot.straight(-25);
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 10);
  // robot.stop();
  robot.straight(11); 
  sys_task::intake_req = 0;
  robot.turn_pt(Angle(-135, Unit::DEGREES));
  sys_task::left_wing_req = 1;
  robot.stop();
  robot.straight(-16);
  sys_task::left_wing_req = 0;
  robot.turn_pt(Angle(175, Unit::DEGREES));
  robot.turn_pt(Angle(-175, Unit::DEGREES));
  // robot.straight(2);
  robot.turn_pt(Angle(2, Unit::DEGREES));
  robot.stop();
  sys_task::intake_rev_req = 1;
  Trajectory1D::set_constraints(MAX_SPEED * 0.55, MAX_ACCEL, 5);
  robot.straight(34.5);
  robot.stop();
  pros::delay(500);
  sys_task::intake_rev_req = 0;
  pros::delay(100);
}

void rush_close() {
  sys_task::intake_req = 1;
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 10);
}

void skills() {

}

void safe_6b() {
  sys_task::intake_req = 1;
  pros::delay(250);
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 15);
  robot.straight(12);
  robot.stop();
  Trajectory1D::set_constraints(MAX_SPEED * 0.65, MAX_ACCEL, 5);
  robot.straight(-26);
  robot.stop();
  // Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 15);
  robot.turn_pt(Angle(-45, Unit::DEGREES));
  // robot.stop();
  sys_task::left_wing_req = 1;
  sys_task::right_wing_req = 1;
  sys_task::intake_req = 0;
  robot.straight(-16);
  sys_task::left_wing_req = 0;
  pros::delay(150);
  // robot.stop();
  robot.turn_pt(Angle(-85, Unit::DEGREES));
  // robot.stop();
  // sys_task::right_wing_req = 1;

  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 50);
  sys_task::right_wing_req = 0;
  robot.straight(-35);
  robot.stop();
  // Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 5);
  robot.straight(11);
  robot.turn_pt(Angle(101, Unit::DEGREES));
  sys_task::intake_rev_req = 1;
  pros::delay(250);
  robot.straight(31);
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 10);
  robot.stop();
  pros::delay(200);
  robot.straight(-8);
  sys_task::intake_rev_req = 0;
  robot.turn_pt(Angle(28, Unit::DEGREES));
  robot.stop();
  sys_task::intake_req = 1;
  robot.straight(57);
  robot.straight(-6);
  robot.stop();
  robot.turn_pt(Angle(160, Unit::DEGREES));
  robot.stop();
  sys_task::intake_req = 0;
  sys_task::intake_speed = 127;
  sys_task::intake_rev_req = 1;
  robot.straight(8);
  sys_task::intake_rev_req = 0;
  sys_task::intake_speed = 127;
  sys_task::intake_req = 1;
  robot.turn_pt(Angle(50, Unit::DEGREES));
  robot.straight(26);
  robot.stop();
  robot.straight(-2);
  robot.turn_pt(Angle(180, Unit::DEGREES));
  robot.stop();
  sys_task::intake_req = 0;
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 50);
  sys_task::front_wings_req = 1;
  sys_task::intake_rev_req = 1;
  pros::delay(200);
  robot.straight(50);
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 15);
  sys_task::front_wings_req = 0;
  sys_task::intake_rev_req = 0;
  pros::delay(150);
  robot.straight(-10);
  robot.stop();
  // sys_task::right_wing_req = 0;
}

} // namespace auton