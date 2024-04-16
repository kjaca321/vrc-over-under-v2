#include "main.h"

namespace auton {

float MAX_SPEED = 67;
float MAX_ACCEL = 130;
#define A Angle
#define V Vector

void rush_6b_mid() {
  // initialize heading and profiles
  robot.set_heading(Angle(69, Unit::DEGREES));
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 15);
  Trajectory2D::set_constraints(MAX_SPEED, MAX_ACCEL, 5, 11.0);

  // push alliance ball to side, rush mid ball
  sys_task::intake_req = 1;
  sys_task::front_wings_req = 1;
  pros::delay(100);
  sys_task::front_wings_req = 0;
  pros::delay(150);
  robot.straight(49);

  // spline back, flick held ball and grab elevation bar ball
  robot.follow_prim(CubicBezier(V(15, 36), V(15, 36), 8.0), -1);
  sys_task::intake_req = 0;
  robot.turn_pt(A(135, Unit::DEGREES));
  sys_task::intake_rev_req = 1;
  pros::delay(200);
  sys_task::intake_rev_req = 0;
  sys_task::intake_req = 1;
  robot.turn_pt(A(-1, Unit::DEGREES));
  robot.straight(32);

  // score balls into side of goal
  robot.follow_prim(
      []() {
        pros::delay(350);
        sys_task::intake_req = 0;
        pros::delay(400);
        sys_task::left_wing_req = 1;
        sys_task::right_wing_req = 1;
        pros::delay(500);
      },
      CubicBezier(V(3, 32), V(-15, 45), 38.0), -1);
  sys_task::right_wing_req = 0;
  sys_task::left_wing_req = 0;
  robot.turn_pt(A(-78, Unit::DEGREES));
  robot.move(-127);
  pros::delay(700);
  robot.stop_fast();

  // turn around, score held ball, back out
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 20);
  sys_task::right_wing_req = 0;
  sys_task::left_wing_req = 0;
  robot.straight(5);
  robot.turn_pt(A(101, Unit::DEGREES));
  sys_task::intake_rev_req = 1;
  pros::delay(250);
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 50);
  robot.straight(15);
  robot.stop_fast();
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 20);
  pros::delay(200);
  robot.straight(-5);
  sys_task::intake_rev_req = 0;

  // grab bottom mid ball
  robot.turn_pt(A(22.3, Unit::DEGREES));
  sys_task::intake_req = 1;
  Trajectory1D::set_constraints(70, 140, 20);
  robot.straight(54);
  robot.straight(-6);

  // turn and score bottom mid ball, back out
  robot.turn_pt(A(135, Unit::DEGREES));
  sys_task::intake_req = 0;
  Trajectory2D::set_constraints(68, 130, 15, 11.0);
  robot.follow_prim(
      []() {
        pros::delay(400);
        sys_task::intake_rev_req = 1;
      },
      CubicBezier(V(25, 35), V(25, 35), 25.0), 1);
  sys_task::intake_rev_req = 0;
  robot.straight(-4);

  // grab far mid ball
  robot.turn_pt(A(0, Unit::DEGREES));
  Trajectory1D::set_constraints(70, 140, 20);
  sys_task::intake_rev_req = 0;
  sys_task::intake_req = 1;
  robot.straight(25);
  robot.straight(-4);
  sys_task::intake_req = 0;

  // turn an dscore far mid ball
  robot.turn_pt(A(-178, Unit::DEGREES));
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 70);
  sys_task::front_wings_req = 1;
  sys_task::intake_rev_req = 1;
  pros::delay(200);
  robot.straight(50);

  // back out
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 20);
  sys_task::front_wings_req = 0;
  sys_task::intake_rev_req = 0;
  pros::delay(150);
  robot.straight(-5);
  robot.stop();
  pros::delay(2);
  robot.stop_fast();
}

void rush_6b_far() {
  // initialize heading and profiles
  robot.set_heading(Angle(45, Unit::DEGREES));
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 15);
  Trajectory2D::set_constraints(MAX_SPEED, MAX_ACCEL, 5, 11.0);

  // push alliance ball to side, rush far mid ball
  sys_task::intake_req = 1;
  sys_task::front_wings_req = 1;
  pros::delay(100);
  sys_task::front_wings_req = 0;
  pros::delay(150);
  robot.straight(55);

  // spline back, flick held ball and grab elevation bar ball
  robot.follow_prim(CubicBezier(V(15, 36), V(15, 36), 8.0), -1);
  sys_task::intake_req = 0;
  robot.turn_pt(A(135, Unit::DEGREES));
  sys_task::intake_rev_req = 1;
  pros::delay(200);
  sys_task::intake_rev_req = 0;
  sys_task::intake_req = 1;
  robot.turn_pt(A(-1, Unit::DEGREES));
  robot.straight(32);

  // score balls into side of goal
  robot.follow_prim(
      []() {
        pros::delay(350);
        sys_task::intake_req = 0;
        pros::delay(400);
        sys_task::left_wing_req = 1;
        sys_task::right_wing_req = 1;
        pros::delay(500);
      },
      CubicBezier(V(3, 32), V(-15, 45), 38.0), -1);
  sys_task::right_wing_req = 0;
  sys_task::left_wing_req = 0;
  robot.turn_pt(A(-78, Unit::DEGREES));
  robot.move(-127);
  pros::delay(700);
  robot.stop_fast();

  // turn around, score held ball, back out
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 20);
  sys_task::right_wing_req = 0;
  sys_task::left_wing_req = 0;
  robot.straight(5);
  robot.turn_pt(A(101, Unit::DEGREES));
  sys_task::intake_rev_req = 1;
  pros::delay(250);
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 50);
  robot.straight(15);
  robot.stop_fast();
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 20);
  pros::delay(200);
  robot.straight(-5);
  sys_task::intake_rev_req = 0;

  // grab bottom mid ball
  robot.turn_pt(A(22.3, Unit::DEGREES));
  sys_task::intake_req = 1;
  Trajectory1D::set_constraints(70, 140, 20);
  robot.straight(54);
  robot.straight(-6);

  // turn and score bottom mid ball, back out
  robot.turn_pt(A(135, Unit::DEGREES));
  sys_task::intake_req = 0;
  Trajectory2D::set_constraints(68, 130, 15, 11.0);
  robot.follow_prim(
      []() {
        pros::delay(400);
        sys_task::intake_rev_req = 1;
      },
      CubicBezier(V(25, 35), V(25, 35), 25.0), 1);
  sys_task::intake_rev_req = 0;
  robot.straight(-4);

  // grab mid ball
  robot.turn_pt(A(0, Unit::DEGREES));
  Trajectory1D::set_constraints(70, 140, 20);
  sys_task::intake_rev_req = 0;
  sys_task::intake_req = 1;
  robot.straight(25);
  robot.straight(-4);
  sys_task::intake_req = 0;

  // turn and score mid ball
  robot.turn_pt(A(-178, Unit::DEGREES));
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 70);
  sys_task::front_wings_req = 1;
  sys_task::intake_rev_req = 1;
  pros::delay(200);
  robot.straight(50);

  // back out
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 20);
  sys_task::front_wings_req = 0;
  sys_task::intake_rev_req = 0;
  pros::delay(150);
  robot.straight(-5);
  robot.stop();
  pros::delay(2);
  robot.stop_fast();
}

void safe_6b() {
  // initialize motion profiles
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 15);
  Trajectory2D::set_constraints(MAX_SPEED, MAX_ACCEL, 15, 11.0);

  // grab elevation bar ball
  sys_task::intake_req = 1;
  pros::delay(300);
  robot.straight(10);

  // score balls into side of goal
  robot.follow_prim(
      []() {
        pros::delay(350);
        sys_task::intake_req = 0;
        pros::delay(400);
        sys_task::left_wing_req = 1;
        sys_task::right_wing_req = 1;
        pros::delay(500);
      },
      CubicBezier(V(3, 32), V(-15, 45), 38.0), -1);
  sys_task::right_wing_req = 0;
  sys_task::left_wing_req = 0;
  robot.turn_pt(A(-78, Unit::DEGREES));
  robot.move(-127);
  pros::delay(700);
  robot.stop_fast();

  // turn around, score held ball, back out
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 20);
  sys_task::right_wing_req = 0;
  sys_task::left_wing_req = 0;
  robot.straight(5);
  robot.turn_pt(A(101, Unit::DEGREES));
  sys_task::intake_rev_req = 1;
  pros::delay(250);
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 50);
  robot.straight(15);
  robot.stop_fast();
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 20);
  pros::delay(200);
  robot.straight(-5);
  sys_task::intake_rev_req = 0;

  // grab bottom mid ball
  robot.turn_pt(A(22.3, Unit::DEGREES));
  sys_task::intake_req = 1;
  Trajectory1D::set_constraints(70, 140, 20);
  robot.straight(54);
  robot.straight(-6);

  // turn and score bottom mid ball, back out
  robot.turn_pt(A(135, Unit::DEGREES));
  sys_task::intake_req = 0;
  Trajectory2D::set_constraints(68, 130, 15, 11.0);
  robot.follow_prim(
      []() {
        pros::delay(400);
        sys_task::intake_rev_req = 1;
      },
      CubicBezier(V(25, 35), V(25, 35), 25.0), 1);
  sys_task::intake_rev_req = 0;
  robot.straight(-2);

  // turn and grab far mid ball
  robot.turn_pt(A(118, Unit::DEGREES));
  Trajectory1D::set_constraints(70, 140, 15);
  robot.straight(-9);
  sys_task::intake_rev_req = 0;
  sys_task::intake_req = 1;
  robot.turn_pt(A(41, Unit::DEGREES));
  robot.straight(24.5);
  robot.stop_fast();
  pros::delay(100);

  // score/push far mid and mid ball together
  sys_task::intake_req = 0;
  robot.turn_pt(A(-179, Unit::DEGREES));
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 70);
  sys_task::front_wings_req = 1;
  sys_task::intake_rev_req = 1;
  pros::delay(200);
  robot.straight(50);

  // back out
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 15);
  sys_task::front_wings_req = 0;
  sys_task::intake_rev_req = 0;
  pros::delay(150);
  robot.straight(-5);
  robot.stop();
  pros::delay(2);
  robot.stop_fast();
}

void safe_6b_touch() {
  // initialize motion profiles
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 15);
  Trajectory2D::set_constraints(MAX_SPEED, MAX_ACCEL, 15, 11.0);

  // grab elevation bar ball
  sys_task::intake_req = 1;
  pros::delay(300);
  robot.straight(10);

  // score balls into side of goal
  robot.follow_prim(
      []() {
        pros::delay(350);
        sys_task::intake_req = 0;
        pros::delay(400);
        sys_task::left_wing_req = 1;
        sys_task::right_wing_req = 1;
        pros::delay(500);
      },
      CubicBezier(V(3, 32), V(-15, 45), 38.0), -1);
  sys_task::right_wing_req = 0;
  sys_task::left_wing_req = 0;
  robot.turn_pt(A(-78, Unit::DEGREES));
  robot.move(-127);
  pros::delay(700);
  robot.stop_fast();

  // turn around, score held ball, back out
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 20);
  sys_task::right_wing_req = 0;
  sys_task::left_wing_req = 0;
  robot.straight(5);
  robot.turn_pt(A(101, Unit::DEGREES));
  sys_task::intake_rev_req = 1;
  pros::delay(250);
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 50);
  robot.straight(15);
  robot.stop_fast();
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 20);
  pros::delay(200);
  robot.straight(-5);
  sys_task::intake_rev_req = 0;

  // grab bottom mid ball
  robot.turn_pt(A(22.3, Unit::DEGREES));
  sys_task::intake_req = 1;
  Trajectory1D::set_constraints(70, 140, 20);
  robot.straight(54);
  robot.straight(-6);

  // turn and score bottom mid ball, back out
  robot.turn_pt(A(135, Unit::DEGREES));
  sys_task::intake_req = 0;
  Trajectory2D::set_constraints(68, 130, 15, 11.0);
  robot.follow_prim(
      []() {
        pros::delay(400);
        sys_task::intake_rev_req = 1;
      },
      CubicBezier(V(25, 35), V(25, 35), 25.0), 1);
  sys_task::intake_rev_req = 0;
  robot.straight(-2);

  // turn and grab far mid ball
  robot.turn_pt(A(118, Unit::DEGREES));
  Trajectory1D::set_constraints(70, 140, 15);
  robot.straight(-9);
  sys_task::intake_rev_req = 0;
  sys_task::intake_req = 1;
  robot.turn_pt(A(41, Unit::DEGREES));
  robot.straight(24.5);
  robot.stop_fast();
  pros::delay(100);

  // score/push far mid and mid ball together
  sys_task::intake_req = 0;
  robot.turn_pt(A(-179, Unit::DEGREES));
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 70);
  sys_task::front_wings_req = 1;
  sys_task::intake_rev_req = 1;
  pros::delay(200);
  robot.straight(50);

  // back out
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 15);
  sys_task::front_wings_req = 0;
  sys_task::intake_rev_req = 0;
  robot.straight(-5);

  //touch bar
  robot.follow_prim(
      []() {
        pros::delay(400);
        sys_task::left_wing_req = 1;
      },
      CubicBezier(V(-40, 32), V(-40, 32), 33.0), -1);
  robot.move(0);
  robot.stop();
}

void safe_4b_touch() {
  // initialize motion profiles
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 15);
  Trajectory2D::set_constraints(MAX_SPEED, MAX_ACCEL, 15, 11.0);

  // grab elevation bar ball
  sys_task::intake_req = 1;
  pros::delay(300);
  robot.straight(10);

  // score balls into side of goal
  robot.follow_prim(
      []() {
        pros::delay(350);
        sys_task::intake_req = 0;
        pros::delay(400);
        sys_task::left_wing_req = 1;
        sys_task::right_wing_req = 1;
        pros::delay(500);
      },
      CubicBezier(V(3, 32), V(-15, 45), 38.0), -1);
  sys_task::right_wing_req = 0;
  sys_task::left_wing_req = 0;
  robot.turn_pt(A(-78, Unit::DEGREES));
  robot.move(-127);
  pros::delay(700);
  robot.stop_fast();

  // turn around, score held ball, back out
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 20);
  sys_task::right_wing_req = 0;
  sys_task::left_wing_req = 0;
  robot.straight(5);
  robot.turn_pt(A(101, Unit::DEGREES));
  sys_task::intake_rev_req = 1;
  pros::delay(250);
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 50);
  robot.straight(15);
  robot.stop_fast();
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 20);
  pros::delay(200);
  robot.straight(-5);
  sys_task::intake_rev_req = 0;

  // grab bottom mid ball
  robot.turn_pt(A(22.3, Unit::DEGREES));
  sys_task::intake_req = 1;
  Trajectory1D::set_constraints(70, 140, 20);
  robot.straight(54);
  robot.straight(-6);

  // turn and score bottom mid ball, back out
  robot.turn_pt(A(135, Unit::DEGREES));
  sys_task::intake_req = 0;
  Trajectory2D::set_constraints(68, 130, 15, 11.0);
  robot.follow_prim(
      []() {
        pros::delay(400);
        sys_task::intake_rev_req = 1;
      },
      CubicBezier(V(25, 35), V(25, 35), 25.0), 1);
  sys_task::intake_rev_req = 0;
  robot.straight(-2);

  //touch bar
  robot.follow_prim(
      []() {
        pros::delay(400);
        sys_task::left_wing_req = 1;
      },
      CubicBezier(V(-40, 32), V(-40, 32), 33.0), -1);
  robot.move(0);
  robot.stop();
}

void safe_close() {
  sys_task::intake_req = 1;
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 10);
  robot.turn_pt(A(35, Unit::DEGREES));
  robot.stop_fast();
  robot.straight(17);
  robot.stop_fast();
  sys_task::intake_req = 0;
  robot.turn_pt(A(45, Unit::DEGREES));
  robot.stop_fast();
  sys_task::left_wing_req = 1;
  robot.straight(-16);
  robot.stop_fast();
  robot.turn_pt(A(20, Unit::DEGREES));
  robot.stop_fast();
  sys_task::left_wing_req = 0;
  sys_task::intake_rev_req = 1;
  robot.turn_pt(A(-90, Unit::DEGREES));
  robot.stop_fast();
  robot.turn_pt(A(-160, Unit::DEGREES));
  robot.stop_fast();
  robot.straight(10);
  robot.stop_fast();
  robot.turn_pt(A(180, Unit::DEGREES));
  robot.stop_fast();
  Trajectory1D::set_constraints(40, 90, 8);
  robot.straight(23);
  sys_task::intake_rev_req = 0;
  robot.stop();

  // robot.turn_pt(A(90, Unit::DEGREES));
  // Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 50);
  // robot.straight(-25);
  // Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 10);
  // // robot.stop();
  // robot.straight(11);
  // sys_task::intake_req = 0;
  // robot.turn_pt(A(-135, Unit::DEGREES));
  // sys_task::left_wing_req = 1;
  // robot.stop();
  // robot.straight(-16);
  // sys_task::left_wing_req = 0;
  // robot.turn_pt(A(175, Unit::DEGREES));
  // robot.turn_pt(A(-175, Unit::DEGREES));
  // // robot.straight(2);
  // robot.turn_pt(A(2, Unit::DEGREES));
  // robot.stop();
  // sys_task::intake_rev_req = 1;
  // Trajectory1D::set_constraints(MAX_SPEED * 0.55, MAX_ACCEL, 5);
  // robot.straight(34.5);
  // robot.stop();
  // pros::delay(500);
  // sys_task::intake_rev_req = 0;
  // pros::delay(100);
}

void rush_close() {
  sys_task::intake_req = 1;
  Trajectory2D::set_constraints(70, 130, 15, 11.0);
  Trajectory1D::set_constraints(70, 140, 15);
  sys_task::front_wings_req = 1;
  pros::delay(200);
  sys_task::front_wings_req = 0;
  robot.follow_prim(CubicBezier(V(24, 41), V(24, 41), 20.0), 1);
  robot.move(0);
  Trajectory2D::set_constraints(70, 110, 15, 11.0);
  pros::delay(100);
  robot.turn_pt(A(21, Unit::DEGREES));
  robot.move(0);
  robot.straight(-36);
  robot.stop_fast();
  robot.turn_pt(A(0, Unit::DEGREES));
  robot.stop_fast();
  robot.straight(-14);
  robot.stop_fast();
  sys_task::intake_req = 0;
  robot.turn_pt(A(90, Unit::DEGREES));
  robot.move(0);
  sys_task::intake_rev_req = 1;
  pros::delay(200);
  robot.turn_pt(A(-58.5, Unit::DEGREES));
  robot.move(0);
  sys_task::intake_rev_req = 0;
  robot.straight(21);
  robot.move(0);
  robot.turn_pt(A(-45, Unit::DEGREES));
  robot.move(0);
  sys_task::left_wing_req = 1;

  robot.follow_prim(
      []() {
        pros::delay(450);
        sys_task::left_wing_req = 0;
      },
      CubicBezier(V(-37, 17), V(-37, 17), 7.0), -1);

  robot.move(0);

  robot.turn_pt(A(90, Unit::DEGREES));
  robot.stop_fast();
  sys_task::intake_rev_req = 1;
  Trajectory1D::set_constraints(50, 95, 8);
  robot.straight(11);
  robot.move(0);

  robot.follow_prim(CubicBezier(V(12, 44), V(12, 44), 34.0), -1);
  sys_task::intake_rev_req = 0;
  robot.stop_fast();
}

void rush_close_wp() {
  sys_task::intake_req = 1;
  Trajectory2D::set_constraints(70, 130, 15, 11.0);
  Trajectory1D::set_constraints(70, 140, 15);
  sys_task::front_wings_req = 1;
  pros::delay(200);
  sys_task::front_wings_req = 0;
  robot.follow_prim(CubicBezier(V(22, 41), V(22, 41), 20.0), 1);
  robot.move(0);
  Trajectory2D::set_constraints(70, 110, 15, 11.0);
  pros::delay(100);
  robot.turn_pt(A(21, Unit::DEGREES));
  robot.move(0);
  robot.straight(-36);
  robot.stop_fast();
  robot.turn_pt(A(0, Unit::DEGREES));
  robot.stop_fast();
  robot.straight(-12.5);
  robot.stop();
  sys_task::intake_req = 0;
  robot.turn_pt(A(90, Unit::DEGREES));
  robot.move(0);
  sys_task::intake_rev_req = 1;
  pros::delay(200);
  robot.turn_pt(A(-59.5, Unit::DEGREES));
  robot.move(0);
  sys_task::intake_rev_req = 0;
  robot.straight(21);
  robot.move(0);
  robot.turn_pt(A(-45, Unit::DEGREES));
  robot.move(0);
  sys_task::left_wing_req = 1;

  robot.follow_prim(
      []() {
        pros::delay(450);
        sys_task::left_wing_req = 0;
      },
      CubicBezier(V(-37, 17), V(-37, 17), 7.0), -1);

  robot.move(0);

  robot.turn_pt(A(90, Unit::DEGREES));
  robot.stop_fast();
  sys_task::intake_rev_req = 1;
  Trajectory1D::set_constraints(35, 90, 8);
  robot.straight(13);
  robot.move(0);

  robot.stop_fast();
  pros::delay(400);
  // robot.follow_prim(CubicBezier(V(12, 44), V(12, 44), 34.0), -1);
  sys_task::intake_rev_req = 0;
  robot.stop_fast();
}

void rush_close_push() {
  sys_task::intake_req = 1;
  Trajectory2D::set_constraints(70, 130, 15, 11.0);
  Trajectory1D::set_constraints(70, 140, 15);
  sys_task::front_wings_req = 1;
  pros::delay(200);
  sys_task::front_wings_req = 0;
  robot.follow_prim(CubicBezier(V(24, 41), V(24, 41), 20.0), 1);
  robot.move(0);
  Trajectory2D::set_constraints(70, 110, 15, 11.0);
  pros::delay(100);
  robot.move(0);
  robot.straight(-4);
  robot.move(0);
  robot.turn_pt(A(90, Unit::DEGREES));
  robot.move(0);
  sys_task::front_wings_req = 1;
  robot.straight(30);
  robot.move(0);
  robot.straight(-13);
  sys_task::front_wings_req = 0;
  robot.move(0);
  robot.turn_pt(A(21, Unit::DEGREES));
  robot.move(0);
  robot.straight(-36);
  robot.stop_fast();
  robot.turn_pt(A(0, Unit::DEGREES));
  robot.stop_fast();
  robot.straight(-12);
  robot.stop_fast();
  sys_task::intake_req = 0;
  robot.turn_pt(A(90, Unit::DEGREES));
  robot.move(0);
  sys_task::intake_rev_req = 1;
  pros::delay(200);
  robot.turn_pt(A(-58.5, Unit::DEGREES));
  robot.move(0);
  sys_task::intake_rev_req = 0;
  robot.straight(21);
  robot.move(0);
  robot.turn_pt(A(-45, Unit::DEGREES));
  robot.move(0);
  sys_task::left_wing_req = 1;

  robot.follow_prim(
      []() {
        pros::delay(450);
        sys_task::left_wing_req = 0;
      },
      CubicBezier(V(-37, 17), V(-37, 17), 7.0), -1);

  robot.move(0);

  robot.turn_pt(A(90, Unit::DEGREES));
  robot.stop_fast();
  sys_task::intake_rev_req = 1;
  Trajectory1D::set_constraints(50, 95, 8);
  robot.turn_pt(A(90, Unit::DEGREES));
  robot.stop_fast();
  robot.straight(15);
  robot.move(0);

  robot.follow_prim(CubicBezier(V(12, 44), V(12, 44), 34.0), -1);
  sys_task::intake_rev_req = 0;
  robot.stop_fast();
}

void skills() {
  skills_start();

  Trajectory2D::set_constraints(67, 130, 20, 11.0);
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 15);
  robot.stop_fast();
  // robot.turn_pt(A(60, Unit::DEGREES));
  robot.stop_fast();
  // robot.turn_pt(A(57, Unit::DEGREES));
  // robot.stop_fast();
  // robot.turn_pt(A(57, Unit::DEGREES));
  // robot.stop_fast();
  robot.straight(18);
  robot.stop_fast();
  sys_task::front_wings_req = 1;
  // robot.turn_swing(A(135, Unit::DEGREES), 1);
  // robot.stop_fast();
  // robot.turn_pt(A(135, Unit::DEGREES));
  // robot.stop_fast();
  // robot.straight(105);
  Trajectory2D::set_constraints(60, 110, 20, 11.0);
  robot.follow_prim(CubicBezier(V(14, 35), V(85, 53), 36.0), 1);
  Trajectory2D::set_constraints(67, 130, 20, 11.0);
  // robot.move(70);
  // pros::delay(200);
  robot.stop_fast();
  sys_task::front_wings_req = 0;
  robot.straight(-4);
  robot.stop_fast();
  pros::delay(5);
  robot.turn_pt(A(-149, Unit::DEGREES));
  robot.stop_fast();
  robot.turn_pt(A(-149, Unit::DEGREES));
  robot.stop_fast();
  robot.straight(45);
  robot.stop_fast();
  // sys_task::front_wings_req = 1;
  robot.stop_fast();
  robot.turn_pt(A(90, Unit::DEGREES));
  robot.stop_fast();
  robot.turn_pt(A(90, Unit::DEGREES));
  robot.stop_fast();

  // sys_task::front_wings_req = 0;
  robot.follow_prim(
      []() {
        pros::delay(600);
        sys_task::front_wings_req = 0;
      },
      CubicBezier(V(-25, 50), V(-25, 50), 30.0), 1);
  robot.stop_fast();
  robot.turn_pt(A(45, Unit::DEGREES));
  robot.stop_fast();
  robot.turn_pt(A(45, Unit::DEGREES));
  robot.stop_fast();

  // robot.straight(35);
  // robot.stop_fast();
  // robot.turn_pt(A(45, Unit::DEGREES));
  // robot.stop_fast();
  // robot.straight(35);
  // robot.stop_fast();
  sys_task::front_wings_req = 0;
  robot.straight(10);
  robot.stop_fast();

  // robot.turn_pt(A(47, Unit::DEGREES));
  // robot.stop_fast();
  // pros::delay(250);

  // sys_task::front_wings_req = 1;
  Trajectory2D::set_constraints(65, 120, 15, 11.0);
  // robot.follow_prim(CubicBezier(V(5, 90), V(-33, 90), 55), 1);
  robot.follow_prim(
      []() {
        pros::delay(300);
        sys_task::front_wings_req = 0;
        pros::delay(400);
        // sys_task::front_wings_req = 1;
      },
      CubicBezier(V(0, 47), V(-45, 57), 34.5), 1);
  // sys_task::front_wings_req = 1;
  robot.stop_fast();
  robot.turn_pt(A(-45, Unit::DEGREES));
  robot.stop_fast();
  robot.move(127);
  pros::delay(350);
  robot.stop_fast();
  robot.move(-90);
  pros::delay(350);
  robot.stop_fast();
  robot.move(127);
  pros::delay(600);
  robot.stop_fast();
  sys_task::front_wings_req = 0;

  robot.straight(-5);
  robot.stop_fast();
  robot.turn_pt(A(-120, Unit::DEGREES));
  robot.stop_fast();
  robot.straight(32);
  robot.stop_fast();
  robot.turn_swing(A(-45, Unit::DEGREES), 1);
  robot.stop_fast();
  robot.straight(3.5);
  robot.stop_fast();
  robot.turn_swing(A(37, Unit::DEGREES), 1);
  robot.stop_fast();
  robot.turn_swing(A(37, Unit::DEGREES), 1);
  robot.stop_fast();

  sys_task::front_wings_req = 1;
  Trajectory1D::set_constraints(70, 140, 60);
  robot.straight(70);
  robot.stop_fast();
  sys_task::front_wings_req = 0;
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 10);
  robot.straight(-8);
  robot.stop_fast();
  sys_task::left_wing_req = 1;
  robot.turn_swing(A(140, Unit::DEGREES), -1);
  robot.stop_fast();
  robot.turn_swing(A(-135, Unit::DEGREES), -1);
  robot.stop_fast();
  robot.turn_swing(A(-135, Unit::DEGREES), -1);
  robot.stop_fast();
  sys_task::right_wing_req = 1;
  Trajectory1D::set_constraints(70, 140, 60);
  robot.straight(-78);
  robot.stop_fast();
  robot.straight(7);
  robot.stop_fast();
  sys_task::left_wing_req = 0;
  sys_task::right_wing_req = 0;
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 10);
  robot.straight(6);
  robot.stop_fast();
  robot.turn_swing(A(-45, Unit::DEGREES), 1);
  robot.stop_fast();
  robot.straight(-2);
  robot.stop_fast();
  robot.turn_swing(A(55, Unit::DEGREES), 1);
  robot.stop_fast();
  sys_task::front_wings_req = 1;
  Trajectory1D::set_constraints(70, 140, 60);
  robot.straight(70);
  robot.stop_fast();
  sys_task::front_wings_req = 0;

  robot.straight(-25);
  robot.stop_fast();

  Trajectory1D::set_constraints(70, 140, 30);
  robot.turn_pt(A(-11, Unit::DEGREES));
  robot.stop_fast();
  sys_task::front_wings_req = 1;
  robot.straight(37);
  robot.stop_fast();
  sys_task::front_wings_req = 0;
  robot.straight(-43);
  robot.stop_fast();
  robot.turn_pt(A(45, Unit::DEGREES));
  robot.stop_fast();
  robot.move(-127);
  pros::delay(1200);
  robot.stop_fast();
  robot.move(40);
  pros::delay(550);
  robot.stop_fast();
  robot.straight(-7);
  robot.stop_fast();
  robot.turn_pt(A(-45, Unit::DEGREES));
  robot.stop_fast();
  Trajectory2D::set_constraints(65, 120, 20, 11.0);
  robot.follow_prim(CubicBezier(V(-50, 40), V(-50, 40), 30), 1);
  sys_task::front_wings_req = 1;
  robot.straight(-8);
  robot.stop_fast();
  robot.turn_pt(A(0, Unit::DEGREES));
  robot.straight(-4);
  robot.stop_fast();
  robot.follow_prim(CubicBezier(V(40, 40), V(40, 40), 20), 1);
  robot.stop_fast();
  robot.follow_prim(CubicBezier(V(30, 60), V(30, 60), 70), 1);
  robot.stop_fast();
  robot.turn_pt(A(120, Unit::DEGREES));
  robot.stop_fast();
  robot.move(127);
  pros::delay(700);
  robot.stop_fast();
  robot.straight(-14);
  robot.move(0);
  robot.move(127);
  pros::delay(700);
  robot.stop_fast();
  sys_task::front_wings_req = 0;
  robot.straight(-22);
  robot.stop_fast();
  robot.turn_pt(A(90, Unit::DEGREES));
  robot.stop_fast();
  robot.turn_pt(A(90, Unit::DEGREES));
  robot.stop_fast();
  robot.straight(-18);
  robot.stop_fast();
  robot.turn_pt(A(-135, Unit::DEGREES));
  robot.stop_fast();
  robot.turn_pt(A(-135, Unit::DEGREES));
  robot.stop_fast();
  // robot.follow_prim(CubicBezier(V(-40, 30), V(-40, 30), 26), -1);
  sys_task::hang_req = 1;
  robot.stop_fast();
  robot.move(127);
  pros::delay(1000);
  robot.stop_fast();
  pros::delay(300);
  sys_task::hang_req = 0;
  robot.stop_fast();

  pros::delay(100000);

  Trajectory2D::set_constraints(65, 115, 30, 11.0);
  // robot.follow_prim(CubicBezier(V(18, 35), V(18, 15), 50), 1);
  // robot.stop_fast();

  pros::delay(100000);

  // Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 40);
  // robot.straight(-16);
  // robot.stop_fast();
  // robot.turn_pt(A(66, Unit::DEGREES));
  // robot.stop_fast();
  // robot.turn_pt(A(66, Unit::DEGREES));
  // robot.stop_fast();
  // robot.straight(-56);
  // robot.stop_fast();

  // robot.turn_pt(A(24, Unit::DEGREES));
  // robot.stop_fast();
  // robot.turn_pt(A(24, Unit::DEGREES));
  // robot.stop_fast();
  // sys_task::front_wings_req = 1;
  // Trajectory1D::set_constraints(70, 140, 60);
  // robot.straight(85);
  // robot.stop_fast();
  // sys_task::front_wings_req = 0;
  // pros::delay(400);
  // Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 10);
  // robot.straight(-22);
  // robot.stop_fast();
  // robot.turn_pt(A(135, Unit::DEGREES));
  // robot.stop_fast();
  // robot.turn_pt(A(135, Unit::DEGREES));
  // robot.stop_fast();
  // sys_task::wings_req = 1;
  // robot.straight(-25);
  // robot.stop_fast();
  // robot.turn_pt(A(-135, Unit::DEGREES));
  // robot.stop_fast();
  // robot.turn_pt(A(-135, Unit::DEGREES));
  // robot.stop_fast();
  // Trajectory1D::set_constraints(70, 140, 60);
  // robot.straight(-63);
  // robot.stop_fast();
  // Trajectory2D::set_constraints(67, 100, 15, 11.0);
  // robot.straight(30);
  // robot.stop_fast();
  // Trajectory1D::set_constraints(70, 140, 60);
  // robot.straight(-63);
  // robot.stop_fast();
  // Trajectory2D::set_constraints(67, 100, 15, 11.0);
  // robot.straight(30);
  // sys_task::wings_req = 0;
  // robot.stop_fast();
  // // robot.turn_pt(A(30, Unit::DEGREES));
  // // robot.stop_fast();
  // // sys_task::front_wings_req = 1;
  // // robot.straight(40);
  // // robot.stop_fast();
  // // sys_task::front_wings_req = 0;
  // // robot.straight(-35);
  // // robot.stop_fast();
  // robot.turn_pt(A(-11, Unit::DEGREES));
  // robot.stop_fast();
  // sys_task::front_wings_req = 1;
  // robot.straight(60);
  // robot.stop_fast();
  // sys_task::front_wings_req = 0;
  // robot.straight(-45);
  // robot.stop_fast();
  // robot.turn_pt(A(45, Unit::DEGREES));
  // robot.stop_fast();
  // robot.move(-127);
  // pros::delay(1150);
  // robot.stop_fast();
  // robot.move(40);
  // pros::delay(550);
  // robot.stop_fast();
  // robot.straight(-7);
  // robot.stop_fast();
  // robot.turn_pt(A(-45, Unit::DEGREES));
  // robot.stop_fast();
  // Trajectory2D::set_constraints(65, 120, 20, 11.0);
  // robot.follow_prim(CubicBezier(V(-50, 40), V(-50, 40), 30), 1);
  // sys_task::front_wings_req = 1;
  // robot.straight(-8);
  // robot.stop_fast();
  // robot.turn_pt(A(0, Unit::DEGREES));
  // robot.straight(-4);
  // robot.stop_fast();
  // robot.follow_prim(CubicBezier(V(40, 40), V(40, 40), 20), 1);
  // robot.stop_fast();
  // robot.follow_prim(CubicBezier(V(30, 60), V(30, 60), 70), 1);
  // robot.stop_fast();
  // robot.turn_pt(A(120, Unit::DEGREES));
  // robot.stop_fast();
  // robot.move(127);
  // pros::delay(700);
  // robot.stop_fast();
  // robot.straight(-14);
  // robot.move(0);
  // robot.move(127);
  // pros::delay(700);
  // robot.stop_fast();
  // sys_task::front_wings_req = 0;
  // robot.straight(-22);
  // robot.stop_fast();
  // robot.turn_pt(A(90, Unit::DEGREES));
  // robot.stop_fast();
  // robot.straight(-12);
  // robot.stop_fast();
  // robot.turn_pt(A(-135, Unit::DEGREES));
  // robot.stop_fast();
  // // robot.follow_prim(CubicBezier(V(-40, 30), V(-40, 30), 26), -1);
  // sys_task::hang_req = 1;
  // robot.stop_fast();
  // robot.move(127);
  // pros::delay(1000);
  // robot.stop_fast();
  // pros::delay(300);
  // sys_task::hang_req = 0;
  // robot.stop_fast();
  // robot.turn_pt(A(100, Unit::DEGREES));
  // robot.stop_fast();
  // robot.straight(55);
  // robot.stop_fast();
  // sys_task::front_wings_req = 0;
  // robot.straight(-10);
  // robot.straight(30);
  // robot.stop_fast();
  // robot.straight(-10);
  // robot.straight(30);
  // robot.stop_fast();
  // robot.straight(-18);
  // robot.stop_fast();

  // sys_task::intake_req = 1;
  // Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 10);
  // // robot.set_heading(A(-45, Unit::DEGREES));
  // robot.straight(-22.5);
  // robot.turn_pt(A(-30, Unit::DEGREES));
  // // robot.stop();
  // Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 50);
  // robot.straight(-37);
  // Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 10);
  // robot.straight(21);
  // robot.stop();
  // robot.turn_pt(A(20+45, Unit::DEGREES));
  // sys_task::intake_req = 0;
  // sys_task::cata_req = 1;
  // robot.straight(-8);
  // sys_task::left_wing_req = 1;
  // robot.turn_pt(A(20+45, Unit::DEGREES));
  // robot.stop();
  // pros::delay(29000);
  // sys_task::cata_req = 0;
  // pros::delay(2);

  // sys_task::left_wing_req = 0;
  // robot.straight(29);
  // robot.stop();
  // robot.turn_pt(A(60+45, Unit::DEGREES));
  // robot.stop();
  // sys_task::intake_rev_req = 1;
  // robot.straight(38);
  // robot.stop();
  // robot.turn_pt(A(45, Unit::DEGREES));
  // robot.stop();
  // robot.straight(-7);
  // robot.stop();
  // Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL+30, 60);
  // sys_task::front_wings_req = 1;
  // // robot.straight(180);
  // robot.turn_pt(A(45, Unit::DEGREES));
  // robot.stop();
  // robot.move(127);
  // pros::delay(2500);
  // Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 10);
  // robot.straight(-15);
  // robot.turn_pt(A(50, Unit::DEGREES));
  // Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL+30, 60);
  // robot.straight(50);
  // Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 10);
  // robot.straight(-25);
  // robot.turn_pt(A(40, Unit::DEGREES));
  // Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL+30, 60);
  // robot.straight(50);
  // Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 10);
  // robot.straight(-25);
  // robot.turn_pt(A(45, Unit::DEGREES));
  // Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL+30, 60);
  // robot.straight(50);
}

void skills_start() {
  sys_task::left_wing_req = 1;
  pros::delay(200);
  sys_task::left_wing_req = 0;
  Trajectory2D::set_constraints(70, 140, 20, 11.0);
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 15);
  robot.follow_prim(CubicBezier(V(0, 18), V(-14, 30), 6.0), -1);
  robot.move(-127);
  pros::delay(400);
  robot.move(0);
  robot.follow_prim(CubicBezier(V(-8, 16), V(-8, 16), 13.0), 2);
  robot.stop_fast();
  robot.turn_pt(A(20.5 + 45, Unit::DEGREES));
  robot.stop_fast();
  pros::delay(20);
  Angle curr = robot.get_heading();
  pros::delay(20);
  robot.stop_fast();
  sys_task::cata_req = 1;
  robot.straight(-8);
  robot.stop_fast();
  sys_task::left_wing_req = 1;
  robot.turn_pt(A(20.5 + 45, Unit::DEGREES));
  robot.stop_fast();
  robot.turn_pt(A(20.5 + 45, Unit::DEGREES));
  robot.set_brake(BrakeType::HOLD);
  robot.stop_fast();
  robot.brake();
  pros::delay(19000 * 1);
  sys_task::cata_req = 0;
  robot.set_brake(BrakeType::COAST);
  robot.stop_fast();
  sys_task::left_wing_req = 0;
  pros::delay(100);
  robot.set_heading(curr);
  pros::delay(2);
}

} // namespace auton
