#include "main.h"

namespace auton {

float MAX_SPEED = 67;
float MAX_ACCEL = 130;
#define A Angle
#define V Vector

void rush_6b_mid() {
  // initialize heading and profiles
  robot.set_heading(Angle(75, Unit::DEGREES));
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 5);
  Trajectory2D::set_constraints(MAX_SPEED, MAX_ACCEL, 5, 11.0);

  // push alliance ball to side, rush mid ball
  sys_task::intake_req = 1;
  sys_task::right_wing_req = 1;
  pros::delay(100);
  sys_task::right_wing_req = 0;
  pros::delay(150);
  robot.straight(51);

  if (fabs(robot.get_heading().get() - 75.0 * M_PI / 180.0) > .15) {
    robot.stop_fast();
    robot.turn_pt(A(75, Unit::DEGREES));
    robot.move(0);
  }

  // robot.turn_pt(A(75, Unit::DEGREES), 2);
  // pros::delay(100000);
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 15);
  // spline back, flick held ball and grab elevation bar ball
  robot.follow_prim(CubicBezier(V(16, 27.5), V(16, 27.5), 23.0), -1);
  sys_task::intake_req = 0;
  robot.turn_pt(A(149, Unit::DEGREES), 1);
  sys_task::intake_rev_req = 1;
  pros::delay(350);
  sys_task::intake_rev_req = 0;
  sys_task::intake_req = 1;
  robot.turn_pt(A(-1, Unit::DEGREES));
  robot.straight(34.5);

  // // score balls into side of goal
  Trajectory2D::set_constraints(42, 130, 5, 11.0);
  robot.follow_prim(
      []() {
        pros::delay(350);
        sys_task::intake_req = 0;
        pros::delay(400);
        sys_task::left_wing_req = 1;
        sys_task::right_wing_req = 1;
        pros::delay(200);
        // sys_task::right_wing_req = 0;
        pros::delay(150);
        sys_task::left_wing_req = 0;
        
      },
      CubicBezier(V(3, 25), V(-8, 36), 27.0), -1);
  // sys_task::right_wing_req = 0;
  sys_task::left_wing_req = 0;
  robot.turn_pt(A(-45, Unit::DEGREES), 1);
  robot.straight(.1);
  // robot.move(0);
  // pros::delay(200);
  Trajectory2D::set_constraints(MAX_SPEED, MAX_ACCEL, 20, 11.0);
  robot.follow_prim([](){
    pros::delay(150);
    // sys_task::right_wing_req = 0;
  },CubicBezier(V(-21, 19.5), V(-21, 19.5), 7), -1);

  Trajectory2D::set_constraints(MAX_SPEED, MAX_ACCEL, 5, 11.0);
  // robot.move(-127);
  // pros::delay(700);
  // robot.stop_fast();

  Trajectory2D::set_constraints(MAX_SPEED, MAX_ACCEL, 15, 11.0);
  // turn around, score held ball, back out
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 20);
  sys_task::right_wing_req = 0;
  sys_task::left_wing_req = 0;
  robot.straight(12);
  sys_task::right_wing_req = 0;
  // robot.turn_pt(A(180, Unit::DEGREES), 1);
  robot.turn_pt(A(135, Unit::DEGREES), 1);
  // robot.straight(-8);
  // robot.turn_rel(A(-176, Unit::DEGREES), 1);
  // sys_task::front_wings_req = 1;
  sys_task::intake_rev_req = 1;
  // pros::delay(250);
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 60);
  // robot.straight(50);
  robot.follow_prim([](){
    pros::delay(150);
    // sys_task::right_wing_req = 0;
  },CubicBezier(V(-16, 21), V(-16, 21), 12), 1);
  sys_task::front_wings_req = 0;
  robot.move(127);
  pros::delay(200);
  // robot.stop_fast();
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 20);
  // pros::delay(200);
  robot.straight(-7);
  sys_task::intake_rev_req = 0;

  // grab bottom mid ball
  robot.turn_pt(A(26, Unit::DEGREES));
  sys_task::intake_req = 1;
  Trajectory1D::set_constraints(70, 140, 20);
  robot.straight(51);
  robot.straight(-5);

  // turn and score bottom mid ball, back out
  robot.turn_pt(A(140, Unit::DEGREES), 1); 
  sys_task::intake_req = 0;
  Trajectory2D::set_constraints(68, 130, 15, 11.0);
  robot.follow_prim(
      []() {
        pros::delay(200);
        sys_task::intake_rev_req = 1;
      },
      CubicBezier(V(25, 35), V(25, 35), 23.0), 1);
      pros::delay(100);
  sys_task::intake_rev_req = 0;
  robot.straight(-8);

  // grab far mid ball
  robot.turn_pt(A(15, Unit::DEGREES), 1);
  Trajectory1D::set_constraints(70, 140, 40);
  sys_task::intake_rev_req = 0;
  sys_task::intake_req = 1;
  robot.straight(21);
  robot.straight(-5);
  // robot.move(0);

  // turn an dscore far mid ball
  sys_task::intake_req = 0;
  sys_task::intake_speed = 127;
  robot.turn_pt(A(-178, Unit::DEGREES), 1);
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 70);
  sys_task::front_wings_req = 1;
  robot.straight([](){pros::delay(50);
  sys_task::intake_rev_req = 1;},50);

  // turn and grab far mid ball
  // Trajectory1D::set_constraints(70, 140, 40);
  // robot.turn_pt(A(130, Unit::DEGREES));
  // Trajectory1D::set_constraints(70, 140, 15);
  // robot.straight(-10);
  // sys_task::intake_rev_req = 0;
  // sys_task::intake_req = 1;
  // robot.turn_pt(A(41, Unit::DEGREES));
  // robot.straight(24.5);
  // robot.straight(-5);

  // // score/push far mid and mid ball together
  // sys_task::intake_req = 0;
  // robot.turn_pt(A(-179, Unit::DEGREES));
  // Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 70);
  // sys_task::front_wings_req = 1;
  // sys_task::intake_rev_req = 1;
  // // pros::delay(200);
  // robot.straight(46);

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
  robot.set_heading(Angle(55, Unit::DEGREES));
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 5);
  Trajectory2D::set_constraints(MAX_SPEED, MAX_ACCEL, 5, 11.0);

  // push alliance ball to side, rush far mid ball
  sys_task::intake_req = 1;
  sys_task::right_wing_req = 1;
  sys_task::front_wings_req = 1;
  pros::delay(100);
  sys_task::right_wing_req = 0;
  sys_task::front_wings_req = 0;
  pros::delay(150);
  robot.straight(54);

  if (fabs(robot.get_heading().get() - 55.0 * M_PI / 180.0) > .15) {
    robot.stop_fast();
    robot.turn_pt(A(55, Unit::DEGREES));
    robot.move(0);
  }

  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 15);
  // robot.move(0);
  // spline back, flick held ball and grab elevation bar ball
  // robot.follow_prim(CubicBezier(V(17, 38), V(17, 38), 15.0), -1);
  
  // robot.follow_prim(CubicBezier(V(21.5, 35), V(21.5, 35), 8.0), -1);
  robot.straight(-44);

  sys_task::intake_req = 0;
  sys_task::intake_speed = 90;
  robot.turn_pt(A(147, Unit::DEGREES));
  sys_task::intake_rev_req = 1;
  pros::delay(350);
  sys_task::intake_rev_req = 0;
  sys_task::intake_speed = 127;
  sys_task::intake_req = 1;  
  robot.turn_pt(A(-40, Unit::DEGREES));
  robot.straight(7);
  robot.turn_pt(A(-1, Unit::DEGREES));
  robot.straight(31);

  // // score balls into side of goal
  Trajectory2D::set_constraints(42, 130, 5, 11.0);
  robot.follow_prim(
      []() {
        pros::delay(350);
        sys_task::intake_req = 0;
        pros::delay(400);
        sys_task::left_wing_req = 1;
        sys_task::right_wing_req = 1;
        pros::delay(200);
        // sys_task::right_wing_req = 0;
        pros::delay(150);
        sys_task::left_wing_req = 0;
        
      },
      CubicBezier(V(3, 25), V(-8, 36), 27.0), -1);

  // sys_task::right_wing_req = 0;
  sys_task::left_wing_req = 0;
  robot.turn_pt(A(-45, Unit::DEGREES), 1);
  robot.straight(.1);
  // robot.move(0);
  // pros::delay(200);
  Trajectory2D::set_constraints(MAX_SPEED, MAX_ACCEL, 20, 11.0);
  robot.follow_prim([](){
    pros::delay(150);
    // sys_task::right_wing_req = 0;
  },CubicBezier(V(-21, 19.5), V(-21, 19.5), 7), -1);

  Trajectory2D::set_constraints(MAX_SPEED, MAX_ACCEL, 5, 11.0);
  // robot.move(-127);
  // pros::delay(700);
  // robot.stop_fast();

  Trajectory2D::set_constraints(MAX_SPEED, MAX_ACCEL, 15, 11.0);
  // turn around, score held ball, back out
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 20);
  sys_task::right_wing_req = 0;
  sys_task::left_wing_req = 0;
  robot.straight(12);
  sys_task::right_wing_req = 0;
  // robot.turn_pt(A(180, Unit::DEGREES), 1);
  robot.turn_pt(A(135, Unit::DEGREES), 1);
  // robot.straight(-8);
  // robot.turn_rel(A(-176, Unit::DEGREES), 1);
  // sys_task::front_wings_req = 1;
  sys_task::intake_rev_req = 1;
  // pros::delay(250);
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 60);
  // robot.straight(50);
  robot.follow_prim([](){
    pros::delay(150);
    // sys_task::right_wing_req = 0;
  },CubicBezier(V(-16, 21), V(-16, 21), 12), 1);
  sys_task::front_wings_req = 0;
  robot.move(127);
  pros::delay(200);
  // robot.stop_fast();
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 20);
  // pros::delay(200);
  robot.straight(-7);
  sys_task::intake_rev_req = 0;


  // grab bottom mid ball
  robot.turn_pt(A(20.5, Unit::DEGREES));
  sys_task::intake_req = 1;
  Trajectory1D::set_constraints(70, 140, 20);
  robot.straight(51);
  robot.straight(-5);

  // turn and score bottom mid ball, back out
  robot.turn_pt(A(95, Unit::DEGREES));
  sys_task::intake_req = 0;
  Trajectory2D::set_constraints(68, 130, 15, 11.0);
  robot.straight(18.5);
  // sys_task::intake_rev_req = 1;
  robot.turn_pt(A(180, Unit::DEGREES));
    sys_task::front_wings_req = 1;
  // robot.follow_prim(
  //     []() {
  //       pros::delay(400);
  //       sys_task::intake_rev_req = 1;
  //     },
  //     CubicBezier(V(45, 28), V(45, 28), 20.0), 1);
  // sys_task::intake_rev_req = 0;
  // robot.straight(-4);

  // grab mid mid ball
  // robot.turn_pt(A(3, Unit::DEGREES));
  // Trajectory1D::set_constraints(70, 140, 40);
  // sys_task::intake_rev_req = 0;
  // sys_task::intake_req = 1;
  // robot.straight(15);
  // robot.straight(-5);
  // robot.move(0);

  // // turn an dscore mid mid ball
  // robot.turn_pt(A(-178, Unit::DEGREES));
  // sys_task::intake_req = 0;
  // Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 70);
  sys_task::front_wings_req = 1;
  sys_task::intake_rev_req = 1;
  robot.move(0);
  pros::delay(200);
  robot.straight(50);

  // turn and grab far mid ball
  // Trajectory1D::set_constraints(70, 140, 40);
  // robot.turn_pt(A(130, Unit::DEGREES));
  // Trajectory1D::set_constraints(70, 140, 15);
  // robot.straight(-10);
  // sys_task::intake_rev_req = 0;
  // sys_task::intake_req = 1;
  // robot.turn_pt(A(41, Unit::DEGREES));
  // robot.straight(24.5);
  // robot.straight(-5);

  // // score/push far mid and mid ball together
  // sys_task::intake_req = 0;
  // robot.turn_pt(A(-179, Unit::DEGREES));
  // Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 70);
  // sys_task::front_wings_req = 1;
  // sys_task::intake_rev_req = 1;
  // // pros::delay(200);
  // robot.straight(46);

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

void rush_6b_flick() {
  // initialize heading and profiles
  robot.set_heading(Angle(-115, Unit::DEGREES));
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 5);
  Trajectory2D::set_constraints(MAX_SPEED, MAX_ACCEL, 5, 11.0);

  // push alliance ball to side, rush far mid ball
  sys_task::intake_req = 1;
  sys_task::left_wing_req = 1;
  pros::delay(100);
  sys_task::left_wing_req = 0;
  pros::delay(150);

  robot.straight(-30);
  robot.turn_pt([](){
    pros::delay(300);
    sys_task::right_wing_req = 1;
  }, A(53, Unit::DEGREES), 1);
  sys_task::right_wing_req = 0;
  robot.straight(5);
  // robot.turn_pt(A(50, Unit::DEGREES), 1);

  
    if (fabs(robot.get_heading().get() - 55.0 * M_PI / 180.0) > .15) {
    robot.stop_fast();
    robot.turn_pt(A(53, Unit::DEGREES));
    robot.move(0);
  }

  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 15);
  // robot.move(0);
  // spline back, flick held ball and grab elevation bar ball
  // robot.follow_prim(CubicBezier(V(17, 38), V(17, 38), 15.0), -1);
  
  // robot.follow_prim(CubicBezier(V(21.5, 35), V(21.5, 35), 8.0), -1);
  robot.straight(-44);

  sys_task::intake_req = 0;
  sys_task::intake_speed = 90;
  robot.turn_pt(A(147, Unit::DEGREES));
  sys_task::intake_rev_req = 1;
  pros::delay(350);
  sys_task::intake_rev_req = 0;
  sys_task::intake_speed = 127;
  sys_task::intake_req = 1;  
  robot.turn_pt(A(-40, Unit::DEGREES));
  robot.straight(7);
  robot.turn_pt(A(-.5, Unit::DEGREES));
  robot.straight(31);

  // // score balls into side of goal
  Trajectory2D::set_constraints(42, 130, 5, 11.0);
  robot.follow_prim(
      []() {
        pros::delay(350);
        sys_task::intake_req = 0;
        pros::delay(400);
        sys_task::left_wing_req = 1;
        sys_task::right_wing_req = 1;
        pros::delay(200);
        // sys_task::right_wing_req = 0;
        pros::delay(150);
        sys_task::left_wing_req = 0;
        
      },
      CubicBezier(V(3, 25), V(-8, 36), 27.0), -1);
  // sys_task::right_wing_req = 0;
  sys_task::left_wing_req = 0;
  robot.turn_pt(A(-45, Unit::DEGREES), 1);
  robot.straight(.1);
  // robot.move(0);
  // pros::delay(200);
  Trajectory2D::set_constraints(MAX_SPEED, MAX_ACCEL, 20, 11.0);
  robot.follow_prim([](){
    pros::delay(150);
    // sys_task::right_wing_req = 0;
  },CubicBezier(V(-21, 19.5), V(-21, 19.5), 7), -1);

  Trajectory2D::set_constraints(MAX_SPEED, MAX_ACCEL, 5, 11.0);
  // robot.move(-127);
  // pros::delay(700);
  // robot.stop_fast();

  Trajectory2D::set_constraints(MAX_SPEED, MAX_ACCEL, 15, 11.0);
  // turn around, score held ball, back out
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 20);
  sys_task::right_wing_req = 0;
  sys_task::left_wing_req = 0;
  robot.straight(12);
  sys_task::right_wing_req = 0;
  // robot.turn_pt(A(180, Unit::DEGREES), 1);
  robot.turn_pt(A(135, Unit::DEGREES), 1);
  // robot.straight(-8);
  // robot.turn_rel(A(-176, Unit::DEGREES), 1);
  // sys_task::front_wings_req = 1;
  sys_task::intake_rev_req = 1;
  // pros::delay(250);
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 60);
  // robot.straight(50);
  robot.follow_prim([](){
    pros::delay(150);
    // sys_task::right_wing_req = 0;
  },CubicBezier(V(-16, 21), V(-16, 21), 12), 1);
  sys_task::front_wings_req = 0;
  robot.move(127);
  pros::delay(200);
  // robot.stop_fast();
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 20);
  // pros::delay(200);
  robot.straight(-7);
  sys_task::intake_rev_req = 0;

  // grab bottom mid ball
  robot.turn_pt(A(20.5, Unit::DEGREES));
  sys_task::intake_req = 1;
  Trajectory1D::set_constraints(70, 140, 20);
  robot.straight(51);
  robot.straight(-5);

  // turn and score bottom mid ball, back out
  robot.turn_pt(A(95, Unit::DEGREES));
  sys_task::intake_req = 0;
  Trajectory2D::set_constraints(68, 130, 15, 11.0);
  robot.straight(15);
  // sys_task::intake_rev_req = 1;
  robot.turn_pt(A(180, Unit::DEGREES));
    sys_task::front_wings_req = 1;
  // robot.follow_prim(
  //     []() {
  //       pros::delay(400);
  //       sys_task::intake_rev_req = 1;
  //     },
  //     CubicBezier(V(45, 28), V(45, 28), 20.0), 1);
  // sys_task::intake_rev_req = 0;
  // robot.straight(-4);

  // grab mid mid ball
  // robot.turn_pt(A(3, Unit::DEGREES));
  // Trajectory1D::set_constraints(70, 140, 40);
  // sys_task::intake_rev_req = 0;
  // sys_task::intake_req = 1;
  // robot.straight(15);
  // robot.straight(-5);
  // robot.move(0);

  // // turn an dscore mid mid ball
  // robot.turn_pt(A(-178, Unit::DEGREES));
  // sys_task::intake_req = 0;
  // Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 70);
  sys_task::front_wings_req = 1;
  sys_task::intake_rev_req = 1;
  robot.move(0);
  pros::delay(200);
  robot.straight(50);

  // turn and grab far mid ball
  // Trajectory1D::set_constraints(70, 140, 40);
  // robot.turn_pt(A(130, Unit::DEGREES));
  // Trajectory1D::set_constraints(70, 140, 15);
  // robot.straight(-10);
  // sys_task::intake_rev_req = 0;
  // sys_task::intake_req = 1;
  // robot.turn_pt(A(41, Unit::DEGREES));
  // robot.straight(24.5);
  // robot.straight(-5);

  // // score/push far mid and mid ball together
  // sys_task::intake_req = 0;
  // robot.turn_pt(A(-179, Unit::DEGREES));
  // Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 70);
  // sys_task::front_wings_req = 1;
  // sys_task::intake_rev_req = 1;
  // // pros::delay(200);
  // robot.straight(46);

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

  // // score balls into side of goal
  // robot.follow_prim(
  //     []() {
  //       pros::delay(350);
  //       sys_task::intake_req = 0;
  //       pros::delay(400);
  //       sys_task::left_wing_req = 1;
  //       sys_task::right_wing_req = 1;
  //       pros::delay(500);
  //     },
  //     CubicBezier(V(3, 32), V(-15, 45), 38.0), -1);
  // sys_task::right_wing_req = 0;
  // sys_task::left_wing_req = 0;
  // sys_task::intake_req = 0;
  // robot.turn_pt(A(-78, Unit::DEGREES));
  // robot.move(-127);
  // pros::delay(700);
  // robot.stop_fast();

  // // score balls into side of goal
  Trajectory2D::set_constraints(42, 130, 5, 11.0);
  robot.follow_prim(
      []() {
        pros::delay(350);
        sys_task::intake_req = 0;
        pros::delay(400);
        sys_task::left_wing_req = 1;
        sys_task::right_wing_req = 1;
        pros::delay(200);
        // sys_task::right_wing_req = 0;
        pros::delay(150);
        sys_task::left_wing_req = 0;
        
      },
      CubicBezier(V(3, 25), V(-8, 36), 24.0), -1);

  // sys_task::right_wing_req = 0;
  sys_task::left_wing_req = 0;
  robot.turn_pt(A(-45, Unit::DEGREES), 1);
  robot.straight(.1);
  // robot.move(0);
  // pros::delay(200);
  sys_task::right_wing_req = 0;
  Trajectory2D::set_constraints(MAX_SPEED, MAX_ACCEL, 20, 11.0);
  robot.follow_prim([](){
    pros::delay(150);
    // sys_task::right_wing_req = 0;
  },CubicBezier(V(-21, 19.5), V(-21, 19.5), 7), -1);

  Trajectory2D::set_constraints(MAX_SPEED, MAX_ACCEL, 5, 11.0);
  // robot.move(-127);
  // pros::delay(700);
  // robot.stop_fast();

  Trajectory2D::set_constraints(MAX_SPEED, MAX_ACCEL, 15, 11.0);
  // turn around, score held ball, back out
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 20);
  sys_task::right_wing_req = 0;
  sys_task::left_wing_req = 0;
  robot.straight(10);
  sys_task::right_wing_req = 0;
  // robot.turn_pt(A(180, Unit::DEGREES), 1);
  robot.turn_pt(A(135, Unit::DEGREES), 1);
  // robot.straight(-8);
  // robot.turn_rel(A(-176, Unit::DEGREES), 1);
  // sys_task::front_wings_req = 1;
  sys_task::intake_rev_req = 1;
  // pros::delay(250);
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 60);
  // robot.straight(50);
  robot.follow_prim([](){
    pros::delay(150);
    // sys_task::right_wing_req = 0;
  },CubicBezier(V(-16, 21), V(-16, 21), 12), 1);
  sys_task::front_wings_req = 0;
  robot.move(127);
  pros::delay(200);
  // robot.stop_fast();
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 20);
  // pros::delay(200);
  robot.straight(-7);
  sys_task::intake_rev_req = 0;

  // grab bottom mid ball
  robot.turn_pt(A(24, Unit::DEGREES));
  sys_task::intake_req = 1;
  Trajectory1D::set_constraints(70, 140, 20);
  robot.straight(51);
  robot.straight(-5);

  // turn and score bottom mid ball, back out
  robot.turn_pt(A(135, Unit::DEGREES), 1);
  sys_task::intake_req = 0;
  Trajectory2D::set_constraints(68, 130, 15, 11.0);
  robot.follow_prim(
      []() {
        pros::delay(400);
        sys_task::intake_rev_req = 1;
      },
      CubicBezier(V(25, 35), V(25, 35), 25.0), 1);
  sys_task::intake_rev_req = 0;
  robot.straight(-3);

  // turn and grab far mid ball
  robot.turn_pt(A(120, Unit::DEGREES), 1);
  Trajectory1D::set_constraints(70, 140, 15);
  robot.straight(-12);
  sys_task::intake_rev_req = 0;
  sys_task::intake_req = 1;
  robot.turn_pt(A(50, Unit::DEGREES));
  robot.straight(24.5);
  robot.straight(-5);

  // score/push far mid and mid ball together
  sys_task::intake_req = 0;
  robot.turn_pt(A(180, Unit::DEGREES), 4);
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 70);
  sys_task::front_wings_req = 1;
  sys_task::intake_rev_req = 1;
  pros::delay(200);
  robot.straight(46);


  // back out
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 15);
  sys_task::front_wings_req = 0;
  sys_task::intake_rev_req = 0;

  robot.straight(-5);
  robot.straight(.1);

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

  // // score balls into side of goal
  // robot.follow_prim(
  //     []() {
  //       pros::delay(350);
  //       sys_task::intake_req = 0;
  //       pros::delay(400);
  //       sys_task::left_wing_req = 1;
  //       sys_task::right_wing_req = 1;
  //       pros::delay(500);
  //     },
  //     CubicBezier(V(3, 32), V(-15, 45), 38.0), -1);
  // sys_task::right_wing_req = 0;
  // sys_task::left_wing_req = 0;
  // sys_task::intake_req = 0;
  // robot.turn_pt(A(-78, Unit::DEGREES));
  // robot.move(-127);
  // pros::delay(700);
  // robot.stop_fast();

  // // score balls into side of goal
  Trajectory2D::set_constraints(42, 130, 5, 11.0);
  robot.follow_prim(
      []() {
        pros::delay(350);
        sys_task::intake_req = 0;
        pros::delay(400);
        sys_task::left_wing_req = 1;
        sys_task::right_wing_req = 1;
        pros::delay(200);
        // sys_task::right_wing_req = 0;
        pros::delay(150);
        sys_task::left_wing_req = 0;
        
      },
      CubicBezier(V(3, 25), V(-8, 36), 24.0), -1);

  // sys_task::right_wing_req = 0;
  sys_task::left_wing_req = 0;
  robot.turn_pt(A(-45, Unit::DEGREES), 1);
  robot.straight(.1);
  // robot.move(0);
  // pros::delay(200);
  sys_task::right_wing_req = 0;
  Trajectory2D::set_constraints(MAX_SPEED, MAX_ACCEL, 20, 11.0);
  robot.follow_prim([](){
    pros::delay(150);
    // sys_task::right_wing_req = 0;
  },CubicBezier(V(-21, 19.5), V(-21, 19.5), 7), -1);

  Trajectory2D::set_constraints(MAX_SPEED, MAX_ACCEL, 5, 11.0);
  // robot.move(-127);
  // pros::delay(700);
  // robot.stop_fast();

  Trajectory2D::set_constraints(MAX_SPEED, MAX_ACCEL, 15, 11.0);
  // turn around, score held ball, back out
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 20);
  sys_task::right_wing_req = 0;
  sys_task::left_wing_req = 0;
  robot.straight(10);
  sys_task::right_wing_req = 0;
  // robot.turn_pt(A(180, Unit::DEGREES), 1);
  robot.turn_pt(A(135, Unit::DEGREES), 1);
  // robot.straight(-8);
  // robot.turn_rel(A(-176, Unit::DEGREES), 1);
  // sys_task::front_wings_req = 1;
  sys_task::intake_rev_req = 1;
  // pros::delay(250);
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 60);
  // robot.straight(50);
  robot.follow_prim([](){
    pros::delay(150);
    // sys_task::right_wing_req = 0;
  },CubicBezier(V(-16, 21), V(-16, 21), 12), 1);
  sys_task::front_wings_req = 0;
  robot.move(127);
  pros::delay(200);
  // robot.stop_fast();
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 20);
  // pros::delay(200);
  robot.straight(-7);
  sys_task::intake_rev_req = 0;

  // grab bottom mid ball
  robot.turn_pt(A(24, Unit::DEGREES));
  sys_task::intake_req = 1;
  Trajectory1D::set_constraints(70, 140, 20);
  robot.straight(51);
  robot.straight(-5);

  // turn and score bottom mid ball, back out
  robot.turn_pt(A(135, Unit::DEGREES), 1);
  sys_task::intake_req = 0;
  Trajectory2D::set_constraints(68, 130, 15, 11.0);
  robot.follow_prim(
      []() {
        pros::delay(400);
        sys_task::intake_rev_req = 1;
      },
      CubicBezier(V(25, 35), V(25, 35), 25.0), 1);
  sys_task::intake_rev_req = 0;
  robot.straight(-3);

  // turn and grab far mid ball
  robot.turn_pt(A(120, Unit::DEGREES), 1);
  Trajectory1D::set_constraints(70, 140, 15);
  robot.straight(-12);
  sys_task::intake_rev_req = 0;
  sys_task::intake_req = 1;
  robot.turn_pt(A(50, Unit::DEGREES));
  robot.straight(24.5);
  robot.straight(-5);

  // score/push far mid and mid ball together
  sys_task::intake_req = 0;
  robot.turn_pt(A(180, Unit::DEGREES), 4);
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 70);
  sys_task::front_wings_req = 1;
  sys_task::intake_rev_req = 1;
  pros::delay(200);
  robot.straight(46);

  // back out
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 15);
  sys_task::front_wings_req = 0;
  sys_task::intake_rev_req = 0;

  //touch bar
  Trajectory2D::set_constraints(70, 130, 15, 11.0);
  Trajectory1D::set_constraints(70, 130, 15);
  robot.follow_prim(CubicBezier(Vector(-33, 17), Vector(-33, 17), 15.0), -1);
  robot.move(0);
  robot.turn_pt(Angle(97, Unit::DEGREES));
  Trajectory1D::set_constraints(50, 130, 15);
  robot.straight(-23);
  robot.stop();
  sys_task::left_wing_req = 1;
  sys_task::front_wings_req = 1;
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

  // // score balls into side of goal
  // robot.follow_prim(
  //     []() {
  //       pros::delay(350);
  //       sys_task::intake_req = 0;
  //       pros::delay(400);
  //       sys_task::left_wing_req = 1;
  //       sys_task::right_wing_req = 1;
  //       pros::delay(500);
  //     },
  //     CubicBezier(V(3, 32), V(-15, 45), 38.0), -1);
  // sys_task::right_wing_req = 0;
  // sys_task::left_wing_req = 0;
  // sys_task::intake_req = 0;
  // robot.turn_pt(A(-78, Unit::DEGREES));
  // robot.move(-127);
  // pros::delay(700);
  // robot.stop_fast();

  // // score balls into side of goal
  Trajectory2D::set_constraints(42, 130, 5, 11.0);
  robot.follow_prim(
      []() {
        pros::delay(350);
        sys_task::intake_req = 0;
        pros::delay(400);
        sys_task::left_wing_req = 1;
        sys_task::right_wing_req = 1;
        pros::delay(200);
        // sys_task::right_wing_req = 0;
        pros::delay(150);
        sys_task::left_wing_req = 0;
        
      },
      CubicBezier(V(3, 25), V(-8, 36), 24.0), -1);

  // sys_task::right_wing_req = 0;
  sys_task::left_wing_req = 0;
  robot.turn_pt(A(-45, Unit::DEGREES), 1);
  robot.straight(.1);
  // robot.move(0);
  // pros::delay(200);
  sys_task::right_wing_req = 0;
  Trajectory2D::set_constraints(MAX_SPEED, MAX_ACCEL, 20, 11.0);
  robot.follow_prim([](){
    pros::delay(150);
    // sys_task::right_wing_req = 0;
  },CubicBezier(V(-21, 19.5), V(-21, 19.5), 7), -1);

  Trajectory2D::set_constraints(MAX_SPEED, MAX_ACCEL, 5, 11.0);
  // robot.move(-127);
  // pros::delay(700);
  // robot.stop_fast();

  Trajectory2D::set_constraints(MAX_SPEED, MAX_ACCEL, 15, 11.0);
  // turn around, score held ball, back out
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 20);
  sys_task::right_wing_req = 0;
  sys_task::left_wing_req = 0;
  robot.straight(10);
  sys_task::right_wing_req = 0;
  // robot.turn_pt(A(180, Unit::DEGREES), 1);
  robot.turn_pt(A(135, Unit::DEGREES), 1);
  // robot.straight(-8);
  // robot.turn_rel(A(-176, Unit::DEGREES), 1);
  // sys_task::front_wings_req = 1;
  sys_task::intake_rev_req = 1;
  // pros::delay(250);
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 60);
  // robot.straight(50);
  robot.follow_prim([](){
    pros::delay(150);
    // sys_task::right_wing_req = 0;
  },CubicBezier(V(-16, 21), V(-16, 21), 12), 1);
  sys_task::front_wings_req = 0;
  robot.move(127);
  pros::delay(200);
  // robot.stop_fast();
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 20);
  // pros::delay(200);
  robot.straight(-7);
  sys_task::intake_rev_req = 0;

  // grab bottom mid ball
  robot.turn_pt(A(24, Unit::DEGREES));
  sys_task::intake_req = 1;
  Trajectory1D::set_constraints(70, 140, 20);
  robot.straight(51);
  robot.straight(-5);

  // turn and score bottom mid ball, back out
  robot.turn_pt(A(135, Unit::DEGREES), 1);
  sys_task::intake_req = 0;
  Trajectory2D::set_constraints(68, 130, 15, 11.0);
  robot.follow_prim(
      []() {
        pros::delay(400);
        sys_task::intake_rev_req = 1;
      },
      CubicBezier(V(25, 35), V(25, 35), 25.0), 1);
  sys_task::intake_rev_req = 0;
  robot.straight(-3);


  // back out
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 15);
  sys_task::front_wings_req = 0;
  sys_task::intake_rev_req = 0;

  //touch bar
  Trajectory2D::set_constraints(70, 130, 15, 11.0);
  Trajectory1D::set_constraints(70, 130, 15);
  robot.follow_prim(CubicBezier(Vector(-33, 17), Vector(-33, 17), 15.0), -1);
  robot.move(0);
  robot.turn_pt(Angle(97, Unit::DEGREES));
  Trajectory1D::set_constraints(50, 130, 15);
  robot.straight(-20);
  robot.stop();
  sys_task::left_wing_req = 1;
  sys_task::front_wings_req = 1;
  robot.stop();
}

void safe_close() {
  sys_task::intake_req = 1;
  pros::delay(300);
  Trajectory2D::set_constraints(60, 110, 5, 11.0);
  Trajectory1D::set_constraints(70, 140, 15);
  robot.straight(-1.5);
  robot.turn_pt(A(58, Unit::DEGREES), 1);
  sys_task::right_wing_req = 1;
  pros::delay(200);
  robot.turn_pt(A(-30, Unit::DEGREES), 1);
  pros::delay(100);
  sys_task::right_wing_req = 0;
  sys_task::intake_req = 0;
  sys_task::intake_rev_req = 1;
  robot.turn_pt(A(30, Unit::DEGREES));
  robot.follow_prim(CubicBezier(V(-9, 27), V(-9, 27), 13.0), 1);
  robot.straight(3);
  // robot.turn_pt(A(0, Unit::DEGREES), 1);
  robot.move(0);
  pros::delay(1000);
  sys_task::intake_rev_req = 0;
  robot.move(0);
  robot.stop();
}

void safe_close_alt() {
  sys_task::intake_req = 1;
  pros::delay(300);
  Trajectory2D::set_constraints(60, 110, 5, 11.0);
  Trajectory1D::set_constraints(70, 140, 15);
  robot.straight(-1.5);
  robot.turn_pt(A(58, Unit::DEGREES), 1);
  sys_task::right_wing_req = 1;
  pros::delay(200);
  robot.turn_pt(A(-30, Unit::DEGREES), 1);
  pros::delay(100);
  sys_task::right_wing_req = 0;
  sys_task::intake_req = 0;
  sys_task::intake_rev_req = 1;
  robot.turn_pt(A(30, Unit::DEGREES));
  robot.straight(5);
  robot.straight(-10);
  robot.turn_pt(A(135, Unit::DEGREES));
  robot.straight(-14);
    robot.turn_pt(Angle(-173, Unit::DEGREES));
  Trajectory1D::set_constraints(55, 130, 15);
  robot.straight(-32);
  robot.stop();
  sys_task::left_wing_req =1;
  sys_task::front_wings_req = 1;
  robot.stop();
  pros::delay(1000);
  sys_task::intake_rev_req = 0;
  robot.move(0);
  robot.stop();
}

void rush_close() {
  // initialize heading and profiles
  robot.set_heading(Angle(-75, Unit::DEGREES));
  Trajectory2D::set_constraints(60, 110, 5, 11.0);
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 5);

  // push alliance ball to side, rush mid ball
  sys_task::intake_req = 1;
  sys_task::front_wings_req = 1;
  pros::delay(100);
  sys_task::front_wings_req = 0;
  pros::delay(150);
  robot.straight(49.5);
  // robot.move(0);
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 5);
  robot.straight(-34);
  // robot.move(0);
  Trajectory1D::set_constraints(70, 140, 15);
  robot.turn_pt(A(5, Unit::DEGREES));
  sys_task::intake_req = 0;
  sys_task::intake_rev_req = 1;
  pros::delay(150);
  // robot.turn_pt(A(0, Unit::DEGREES), 1);
  robot.straight(-6);
  robot.move(0);
  pros::delay(200);
  robot.turn_pt(A(61, Unit::DEGREES), 2);
  sys_task::right_wing_req = 1;
  robot.turn_pt(A(-30, Unit::DEGREES),1);
  pros::delay(100);
  sys_task::right_wing_req = 0;
  sys_task::intake_req = 0;
  sys_task::intake_rev_req = 1;
  robot.turn_pt(A(30, Unit::DEGREES));
  robot.follow_prim([](){pros::delay(500); sys_task::front_wings_req = 1;}, CubicBezier(V(-7, 31.5), V(-7, 31.5), 13.0), 1);
  robot.straight(2);
  robot.move(0);
  sys_task::front_wings_req = 0;
  pros::delay(500);
  sys_task::intake_rev_req = 0;
  robot.straight(-23);
  robot.move(0);
  // robot.straight(.1);
}

void rush_close_far() {
  // initialize heading and profiles
  robot.set_heading(Angle(-55, Unit::DEGREES));
  Trajectory2D::set_constraints(60, 110, 5, 11.0);
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 5);

  // push alliance ball to side, rush mid ball
  sys_task::intake_req = 1;
  sys_task::front_wings_req = 1;
  pros::delay(100);
  sys_task::front_wings_req = 0;
  pros::delay(150);
  robot.straight(54.5);
  // robot.move(0);
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 5);
  robot.straight(-38);
  // robot.move(0);
  Trajectory1D::set_constraints(70, 140, 15);
  robot.turn_pt(A(5, Unit::DEGREES));
  sys_task::intake_req = 0;
  sys_task::intake_rev_req = 1;
  pros::delay(150);
  // robot.turn_pt(A(0, Unit::DEGREES), 1);
  robot.straight(-4);
  robot.move(0);
  pros::delay(200);
  robot.turn_pt(A(61, Unit::DEGREES), 2);
  sys_task::right_wing_req = 1;
  robot.turn_pt(A(-30, Unit::DEGREES),1);
  pros::delay(100);
  sys_task::right_wing_req = 0;
  sys_task::intake_req = 0;
  sys_task::intake_rev_req = 1;
  robot.turn_pt(A(30, Unit::DEGREES));
  robot.follow_prim([](){pros::delay(500); sys_task::front_wings_req = 1;}, CubicBezier(V(-7, 31.5), V(-7, 31.5), 13.0), 1);
  robot.straight(2);
  robot.move(0);
  sys_task::front_wings_req = 0;
  pros::delay(500);
  sys_task::intake_rev_req = 0;
  robot.straight(-23);
  robot.move(0);
  // robot.straight(.1);
}

void rush_close_wp() {
  // initialize heading and profiles
  robot.set_heading(Angle(-75, Unit::DEGREES));
  Trajectory2D::set_constraints(60, 110, 5, 11.0);
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 5);

  // push alliance ball to side, rush mid ball
  sys_task::intake_req = 1;
  sys_task::front_wings_req = 1;
  pros::delay(100);
  sys_task::front_wings_req = 0;
  pros::delay(150);
  robot.straight(49.5);
  // robot.move(0);
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 5);
  robot.straight(-34);
  // robot.move(0);
  Trajectory1D::set_constraints(70, 140, 15);
  robot.turn_pt(A(5, Unit::DEGREES));
  sys_task::intake_req = 0;
  sys_task::intake_rev_req = 1;
  pros::delay(150);
  // robot.turn_pt(A(0, Unit::DEGREES), 1);
  robot.straight(-6);
  robot.move(0);
  pros::delay(200);
  robot.turn_pt(A(61, Unit::DEGREES), 2);
  sys_task::right_wing_req = 1;
  robot.turn_pt(A(-30, Unit::DEGREES),1);
  pros::delay(100);
  sys_task::right_wing_req = 0;
  sys_task::intake_req = 0;
  sys_task::intake_rev_req = 1;
  robot.turn_pt(A(30, Unit::DEGREES));
  robot.follow_prim([](){pros::delay(500); sys_task::front_wings_req = 1;}, CubicBezier(V(-7, 31.5), V(-7, 31.5), 13.0), 1);
  robot.move(0);
  sys_task::front_wings_req = 0;
  pros::delay(500);
  sys_task::intake_rev_req = 0;
  robot.move(0);
}

void rush_close_push() {
  // initialize heading and profiles
  robot.set_heading(Angle(-75, Unit::DEGREES));
  Trajectory2D::set_constraints(60, 110, 5, 11.0);
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 5);

  // push alliance ball to side, rush mid ball
  sys_task::intake_req = 1;
  sys_task::front_wings_req = 1;
  pros::delay(100);
  sys_task::front_wings_req = 0;
  pros::delay(150);
  robot.straight(49.5);
  robot.straight(-4);
  robot.turn_pt(A(0, Unit::DEGREES));
  sys_task::front_wings_req = 1;
  robot.straight(26);
  robot.straight(-8.5);
  sys_task::front_wings_req = 0;
  // robot.move(0);
  robot.turn_pt(Angle(-75, Unit::DEGREES));
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 5);
  robot.straight(-28);
  // robot.move(0);
  Trajectory1D::set_constraints(70, 140, 15);
  robot.turn_pt(A(5, Unit::DEGREES));
  sys_task::intake_req = 0;
  sys_task::intake_rev_req = 1;
  pros::delay(150);
  // robot.turn_pt(A(0, Unit::DEGREES), 1);
  robot.straight(-5);
  robot.move(0);
  pros::delay(200);
  robot.turn_pt(A(61, Unit::DEGREES), 2);
  sys_task::right_wing_req = 1;
  robot.turn_pt(A(-30, Unit::DEGREES),1);
  pros::delay(100);
  sys_task::right_wing_req = 0;
  sys_task::intake_req = 0;
  sys_task::intake_rev_req = 1;
  robot.turn_pt(A(30, Unit::DEGREES));
  robot.follow_prim([](){pros::delay(500); sys_task::front_wings_req = 1;}, CubicBezier(V(-7, 31.5), V(-7, 31.5), 13.0), 1);
  robot.straight(2);
  robot.move(0);
  sys_task::front_wings_req = 0;
  pros::delay(500);
  sys_task::intake_rev_req = 0;
  robot.straight(-23);
  robot.move(0);
  // robot.straight(.1);
}

void skills_start() {
  //initialize starting heading and motion profiles
  robot.set_heading(A(-45, Unit::DEGREES));
  sys_task::intake_req = 1;
  Trajectory2D::set_constraints(70, 130, 5, 11.0);
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 15);
  
  //deploy intake, score preloads
  // sys_task::left_wing_req = 1;
  // pros::delay(200);
  // sys_task::left_wing_req = 0;
  robot.follow_prim(CubicBezier(V(-23, 20), V(-23, 20), 17.0), -1);

  sys_task::intake_req = 0;
  //setup and start loading
  sys_task::intake_rev_req  = 1;
  robot.follow_prim(CubicBezier(V(10, 18), V(10, 18), 13.0), 1);
  robot.turn_pt(A(-23, Unit::DEGREES));
  robot.set_brake(BrakeType::HOLD);
  robot.move(0);
  robot.brake();
  sys_task::left_wing_req = 1;
  pros::delay(3500);
  robot.set_brake(BrakeType::COAST);
  robot.move(0);
  sys_task::left_wing_req = 0;
}

void skills() {
  //run skills macro, initilaize motion profiles
  skills_start();
  Trajectory2D::set_constraints(67, 130, 5, 11.0);
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 15);
  sys_task::front_wings_req = 0;

  for (int i = 0; i < 3; i++) {
    Trajectory2D::set_constraints(50, 120, 5, 11.0);
    robot.follow_prim([](){
      pros::delay(600);
      sys_task::front_wings_req = 1;
    },CubicBezier(V(20,115), V(60, 98), 25), 1);
    robot.move(127);
    pros::delay(400);
    sys_task::front_wings_req = 0;
    robot.turn_pt(A(90, Unit::DEGREES), 1);
    robot.straight(-6);
    robot.move(127);
    pros::delay(600);
    robot.straight(-4);
    robot.turn_pt(A(45, Unit::DEGREES), 1);
    // robot.move(127);
    // pros::delay(400);
    // robot.turn_pt(A(90, Unit::DEGREES), 1);
    Trajectory2D::set_constraints(67, 130, 5, 11.0);
    robot.straight(-36);
    robot.turn_pt(A(1.5, Unit::DEGREES), 0);
    robot.straight(-50);
    // robot.follow_prim(CubicBezier(V(4, 28), V(-60, 38), 25), -1);
    // robot.straight(-5);
    if (i==2) {
      robot.turn_pt(A(-34, Unit::DEGREES), 1);
      robot.straight(-17);
      sys_task::left_wing_req = 1;
      robot.turn_pt(A(-23, Unit::DEGREES), 0);
      robot.set_brake(BrakeType::HOLD);
      robot.move(0);
      robot.brake();
      pros::delay(8000);
      robot.set_brake(BrakeType::COAST);
      robot.move(0);
      sys_task::left_wing_req = 0;
    }
    else {
      robot.turn_pt(A(-36, Unit::DEGREES), 1);
      robot.straight(-15);
      sys_task::left_wing_req = 1;
      robot.turn_pt(A(-23, Unit::DEGREES), 0);
      robot.set_brake(BrakeType::HOLD);
      robot.move(0);
      robot.brake();
      pros::delay(3600);
      robot.set_brake(BrakeType::COAST);
      robot.move(0);
      sys_task::left_wing_req = 0;
    }
  }

  robot.turn_pt(A(-21, Unit::DEGREES), 1);
  Trajectory2D::set_constraints(46, 120, 5, 11.0);
  robot.follow_prim([](){
    pros::delay(600);
    sys_task::front_wings_req = 1;
  },CubicBezier(V(20,115), V(60, 98), 25), 1);
  robot.move(127);
  pros::delay(400);
  sys_task::front_wings_req = 0;
  robot.turn_pt(A(90, Unit::DEGREES), 1);
  robot.straight(-6);
  robot.move(127);
  pros::delay(600);
  robot.straight(-4);
  robot.turn_pt(A(-135, Unit::DEGREES), 1);
  robot.straight(30);
  sys_task::hang_req = 1;
  robot.turn_pt(A(180, Unit::DEGREES), 1);
  robot.move(127);
  pros::delay(1000);
  sys_task::hang_req = 0;
  robot.move(0);
  robot.stop();
}

void driver_skills() {
  //run skills macro, initilaize motion profiles
  skills_start();
  Trajectory2D::set_constraints(67, 130, 5, 11.0);
  Trajectory1D::set_constraints(MAX_SPEED, MAX_ACCEL, 15);
  sys_task::front_wings_req = 0;

  for (int i = 0; i < 3; i++) {
    Trajectory2D::set_constraints(50, 120, 5, 11.0);
    robot.follow_prim([](){
      pros::delay(600);
      sys_task::front_wings_req = 1;
    },CubicBezier(V(20,115), V(60, 98), 25), 1);
    robot.move(127);
    pros::delay(400);
    sys_task::front_wings_req = 0;
    robot.turn_pt(A(90, Unit::DEGREES), 1);
    robot.straight(-6);
    robot.move(127);
    pros::delay(600);
    robot.straight(-4);
    robot.turn_pt(A(45, Unit::DEGREES), 1);
    // robot.move(127);
    // pros::delay(400);
    // robot.turn_pt(A(90, Unit::DEGREES), 1);
    Trajectory2D::set_constraints(67, 130, 5, 11.0);
    robot.straight(-36);
    robot.turn_pt(A(1.5, Unit::DEGREES), 0);
    robot.straight(-50);
    // robot.follow_prim(CubicBezier(V(4, 28), V(-60, 38), 25), -1);
    // robot.straight(-5);
    if (i==2) {
      robot.turn_pt(A(-34, Unit::DEGREES), 1);
      robot.straight(-17);
      sys_task::left_wing_req = 1;
      robot.turn_pt(A(-23, Unit::DEGREES), 0);
      robot.set_brake(BrakeType::HOLD);
      robot.move(0);
      robot.brake();
      pros::delay(8000);
      robot.set_brake(BrakeType::COAST);
      robot.move(0);
      sys_task::left_wing_req = 0;
    }
    else {
      robot.turn_pt(A(-36, Unit::DEGREES), 1);
      robot.straight(-15);
      sys_task::left_wing_req = 1;
      robot.turn_pt(A(-23, Unit::DEGREES), 0);
      robot.set_brake(BrakeType::HOLD);
      robot.move(0);
      robot.brake();
      pros::delay(3600);
      robot.set_brake(BrakeType::COAST);
      robot.move(0);
      sys_task::left_wing_req = 0;
    }
  }

  robot.turn_pt(A(-21, Unit::DEGREES), 1);
  Trajectory2D::set_constraints(46, 120, 5, 11.0);
  // robot.follow_prim([](){
  //   pros::delay(600);
  //   sys_task::front_wings_req = 1;
  // },CubicBezier(V(20,115), V(60, 98), 25), 1);
  // robot.move(127);
  // pros::delay(400);
  // sys_task::front_wings_req = 0;
  // robot.turn_pt(A(90, Unit::DEGREES), 1);
  // robot.straight(-6);
  // robot.move(127);
  // pros::delay(600);
  // robot.straight(-4);
  // robot.turn_pt(A(-135, Unit::DEGREES), 1);
  // robot.straight(30);
  // sys_task::hang_req = 1;
  // robot.turn_pt(A(180, Unit::DEGREES), 1);
  // robot.move(127);
  // pros::delay(1000);
  // sys_task::hang_req = 0;
  robot.move(0);
  // robot.stop();
}



} // namespace auton
