#include "main.h"

void initialize() {
  robot.set_controller_tuning("arcade", 1, "exponential", 0.02, 10, 0.95);
  pros::lcd::initialize();
  // auton::run_selection();
  robot.setup();
  master.rumble("-");
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  run_auton_sequence();

  math::Angle desired_heading(180, Unit::DEGREES);

  float targ = desired_heading.radians().get();
  float curr = robot.get_heading().get();

  math::Angle raw_ang_dist =
      math::Angle(math::Math::find_min_angle(targ, curr), math::Unit::RADIANS);

  float err = raw_ang_dist.get();
  float err_deg = raw_ang_dist.degrees().get();

  // float i = 600, f = 20, p = 0.843, k = 4.62;
  float i = 600, f = 47, p = 1.08, k = 3.68;

  float kp = ((f - i) * std::pow(fabs(err_deg), p)) /
                 (std::pow(fabs(err_deg), p) + std::pow(k, p)) +
             i;

  // float kp = 54;

  /**
  angle - kp
  1-5 -
  30 - 100
  45 - 80
  90 - 65
  135 - 60
  180 - 54



  */
  float kd = 400;
  float prev = 0;
  float min = 15;
  float tol = 0.015;
  float timeout = 0;
  float maxtime = 2;

  Trajectory1D::set_constraints(67, 180, 5);

  while (1) {
    targ = desired_heading.radians().get();
    curr = robot.get_heading().get();
    math::Angle raw_ang_dist = math::Angle(
        math::Math::find_min_angle(targ, curr), math::Unit::RADIANS);

    err = raw_ang_dist.get();
    float der = (err - prev);
    prev = err;

    float out = kp * err + kd * der;
    if (fabs(out) < min)
      out = Math::sgn(out) * min;
    robot.move_left(out);
    robot.move_right(-out);
    pros::lcd::print(1, "err: %f", err * 180 / M_PI);

    if (fabs(err) < tol)
      timeout++;
    else
      timeout = 0;

    if (timeout >= maxtime)
      break;

    pros::delay(10);
  }

  robot.stop();

  kill_auton_sequence();
}

void opcontrol() {
  run_driver_sequence();
  while (1) {
    std::cout << "vel: " << (robot.get_left_vel() + robot.get_right_vel()) / 2
              << std::endl;
    pros::delay(25);
  }
}