/**
 * \file driver.cpp
 *
 * Driver class, controls all chassis movements, both autonomous and controlled.
 *
 * @author 3135B
 */

#include "library/api/control/driver.hpp"

namespace lib::control {

Driver::Driver(std::vector<int> left_ports, std::vector<int> right_ports,
               float wheel, float speed, int imu1, float trackw)
    : PositionTracker(left_ports, right_ports, wheel, speed, imu1),
      control_type("arcade"), curve(1), map_type("logistic"), accel_time(0.01),
      brake_thresh(0), trackwidth(trackw), turn_sens(1), left_y_error(0),
      left_y_output(0), left_x_error(0), left_x_output(0), right_y_error(0),
      right_y_output(0), right_x_error(0), right_x_output(0), left_velocity(0),
      right_velocity(0), left_final_velocity(0), right_final_velocity(0),
      driver_max_velocity(127) {}

void Driver::straight(float distance) {
  float dir = math::Math::sgn(distance);
  distance = std::abs(distance);
  Trajectory1D path(distance);
  float kv = 1.3, ka = 0.02, kp = .7;
  for (math::Pose1D pose : path.get()) {
    float des_dist = pose.pos;
    float des_vel = pose.vel;
    float des_acc = pose.acc;

    float ff = 0;
    for (int n = 0; n <= 7; n++) {
      ff += args[n] * pow(des_vel, n);
    }

    float out = ff + ka * des_acc +
                kp * (des_vel - (get_left_vel() + get_right_vel()) / 2);
    move(dir * out);
    pros::delay(10);
  }
}

void Driver::straight(void (*sub)(void), float distance) {
  float dir = math::Math::sgn(distance);
  distance = std::abs(distance);
  Trajectory1D path(distance);
  float kv = 1.3, ka = 0.03, kp = .7;
  pros::Task inside = pros::Task(sub);
  for (math::Pose1D pose : path.get()) {
    float des_dist = pose.pos;
    float des_vel = pose.vel;
    float des_acc = pose.acc;
    float out = kv * des_vel + ka * des_acc +
                kp * (des_vel - (get_left_vel() + get_right_vel()) / 2);
    move(dir * out);
    pros::delay(10);
  }

  if (inside.get_state() == pros::E_TASK_STATE_RUNNING)
    inside.remove();
}

void Driver::follow_feed(Trajectory2D trajectory, int direction) {
  float n = 2;
  int lin_dir, ang_dir;

  // direction argument preferences for modifying paths on the fly
  if (direction == 1)
    lin_dir = 1, ang_dir = 1;
  if (direction == -1)
    lin_dir = -1, ang_dir = 1;
  if (direction == 2)
    lin_dir = 1, ang_dir = -1;
  if (direction == -2)
    lin_dir = -1, ang_dir = -1;

  set_relative_position(math::Vector(0, 0));
  set_relative_heading(math::Angle(0, math::Unit::RADIANS));

  for (math::Pose2D pose : trajectory.get()) {
    float des_lin_vel = pose.linear_vel * lin_dir;
    float des_ang_vel = pose.angular_vel * ang_dir;
    float des_acc = pose.linear_acc * lin_dir;
    float des_x = pose.x, des_y = pose.y;

    float b = 1, zeta = 0.;

    float kv = 1.5, ka = 0.0, kp = .0;
    float w = 1;

    math::Angle des_heading = pose.heading;
    math::Vector curr_pos = get_relative_position();
    math::Angle curr_heading = get_relative_heading();

    float err_x = des_x - curr_pos.x, err_y = des_y - curr_pos.y;
    math::Angle err_heading = des_heading - curr_heading;

    float err_x_local =
        cos(curr_heading.get()) * err_x + sin(curr_heading.get()) * err_y;
    float err_y_local =
        cos(curr_heading.get()) * err_y - sin(curr_heading.get()) * err_x;

    float gain =
        2 * zeta *
        sqrt(des_ang_vel * des_ang_vel + b * des_lin_vel * des_lin_vel);

    float vel = des_lin_vel * cos(err_heading.get()) + gain * err_x_local;
    float omega = des_ang_vel + gain * err_heading.get() +
                  b * des_lin_vel * sin(err_heading.get()) * err_y_local /
                      err_heading.get();

    float left_control = vel + w * omega * trackwidth / 2;
    float right_control = vel - w * omega * trackwidth / 2;

    /* apply tuning factors to left and right control outputs, scaled based on a
     * gaussian distribution curve, where 'kv' is the feedforward velocity
     * factor, 'ka' is the feedforward acceleration factor, and 'kp' is the
     * feedback factor */
    float left_out = kv * left_control;
    float right_out = kv * right_control;

    move_left(left_out);
    move_right(right_out);
    pros::delay(10);
  }
}

void Driver::follow_prim(Trajectory2D trajectory, int direction) {
  // tuning factors for path following
  float kv = 1.7, ka = 0.05, kp = .7;
  float w = 2.5;
  float time = 0;
  int lin_dir, ang_dir;

  // direction argument preferences for modifying paths on the fly
  if (direction == 1)
    lin_dir = 1, ang_dir = 1;
  if (direction == -1)
    lin_dir = -1, ang_dir = 1;
  if (direction == 2)
    lin_dir = 1, ang_dir = -1;
  if (direction == -2)
    lin_dir = -1, ang_dir = -1;

  /* iterating through the trajectory of robot poses (position, linear velocity,
   * heading, angular velocity, acceleration) */
  for (math::Pose2D pose : trajectory.get()) {

    // retrieve desired velocities and acceleration from current pose
    float des_lin_vel = pose.linear_vel * lin_dir;
    float des_ang_vel = pose.angular_vel * ang_dir;
    float des_acc = pose.linear_acc * lin_dir;

    /* convert linear and angular velocities to final left and right wheel
     * velocities, from the differential drive kinematics, where 'w' is the
     * responsiveness to desired change in heading and angular velocity */
    float left_control = des_lin_vel + w * des_ang_vel * trackwidth / 2;
    float right_control = des_lin_vel - w * des_ang_vel * trackwidth / 2;

    /* apply tuning factors to left and right control outputs, scaled based on a
     * gaussian distribution curve, where 'kv' is the feedforward velocity
     * factor, 'ka' is the feedforward acceleration factor, and 'kp' is the
     * feedback factor */

    float left_ff = 0, right_ff = 0;
    for (int n = 0; n <= 7; n++) {
      left_ff += args[n] * pow(left_control, n);
      right_ff += args[n] * pow(right_control, n);
    }

    float left_out =
        left_ff + ka * des_acc + kp * (left_control - get_left_vel());
    float right_out =
        right_ff + ka * des_acc + kp * (right_control - get_right_vel());

    move_left(left_out);
    move_right(right_out);
    pros::delay(10);
  }
}

void Driver::follow_prim(void (*sub)(void), Trajectory2D trajectory,
                         int direction) {

  pros::Task inside = pros::Task(sub);
  // tuning factors for path following
  float kv = 1.7, ka = 0.05, kp = .7;
  float w = 2.5;
  float time = 0;
  int lin_dir, ang_dir;

  // direction argument preferences for modifying paths on the fly
  if (direction == 1)
    lin_dir = 1, ang_dir = 1;
  if (direction == -1)
    lin_dir = -1, ang_dir = 1;
  if (direction == 2)
    lin_dir = 1, ang_dir = -1;
  if (direction == -2)
    lin_dir = -1, ang_dir = -1;

  /* iterating through the trajectory of robot poses (position, linear velocity,
   * heading, angular velocity, acceleration) */
  for (math::Pose2D pose : trajectory.get()) {

    // retrieve desired velocities and acceleration from current pose
    float des_lin_vel = pose.linear_vel * lin_dir;
    float des_ang_vel = pose.angular_vel * ang_dir;
    float des_acc = pose.linear_acc * lin_dir;

    /* convert linear and angular velocities to final left and right wheel
     * velocities, from the differential drive kinematics, where 'w' is the
     * responsiveness to desired change in heading and angular velocity */
    float left_control = des_lin_vel + w * des_ang_vel * trackwidth / 2;
    float right_control = des_lin_vel - w * des_ang_vel * trackwidth / 2;

    /* apply tuning factors to left and right control outputs, scaled based on a
     * gaussian distribution curve, where 'kv' is the feedforward velocity
     * factor, 'ka' is the feedforward acceleration factor, and 'kp' is the
     * feedback factor */

    float left_ff = 0, right_ff = 0;
    for (int n = 0; n <= 7; n++) {
      left_ff += args[n] * pow(left_control, n);
      right_ff += args[n] * pow(right_control, n);
    }

    float left_out =
        left_ff + ka * des_acc + kp * (left_control - get_left_vel());
    float right_out =
        right_ff + ka * des_acc + kp * (right_control - get_right_vel());

    move_left(left_out);
    move_right(right_out);
    pros::delay(10);
  }
  if (inside.get_state() == pros::E_TASK_STATE_RUNNING)
    inside.remove();
}

void Driver::turn_pt(math::Angle desired_heading) {
  float targ = desired_heading.radians().get();
  float curr = get_heading().get();
  math::Angle raw_ang_dist =
      math::Angle(math::Math::find_min_angle(targ, curr), math::Unit::RADIANS);

  float err = raw_ang_dist.get();
  float err_deg = raw_ang_dist.degrees().get();

  float min = 140, x = fabs(err_deg);
  bool skip = 0;
  if (x == 0) {
    x = 0.00001;
    skip = 1;
  }

  float kp =
      fmax(min, prop_filter[0] + prop_filter[1] * x + prop_filter[2] * 1 / x +
                    prop_filter[3] * pow(x, prop_filter[4]));

  float kd = derivative_filter[0] + derivative_filter[1] * x +
             derivative_filter[2] * x * x + derivative_filter[3] * x * x * x +
             derivative_filter[4] * x * x * x * x;

  float prev = 0, tol = 0.008, tol2 = .007, timeout = 0, timeout2 = 0,
        maxtime = 2;
  float ang, prev_ang = 0;

  while (1) {
    if (skip)
      break;
    targ = desired_heading.radians().get();
    curr = get_heading().get();
    math::Angle raw_ang_dist = math::Angle(
        math::Math::find_min_angle(targ, curr), math::Unit::RADIANS);

    err = raw_ang_dist.get();
    float der = (err - prev);
    prev = err;

    float out = kp * err + kd * der;
    move_left(out);
    move_right(-out);

    ang = curr;
    float del_ang = ang - prev_ang;
    prev_ang = ang;

    if (fabs(err) < tol)
      timeout++;
    else
      timeout = 0;

    if (fabs(del_ang) < tol2)
      timeout2++;
    else
      timeout2 = 0;

    if (timeout >= maxtime || timeout2 >= maxtime + 8)
      break;

    pros::delay(10);
  }
}

void Driver::turn_swing(math::Angle desired_heading, int direction) {
  float targ = desired_heading.radians().get();
  float curr = get_heading().get();
  math::Angle raw_ang_dist =
      math::Angle(math::Math::find_min_angle(targ, curr), math::Unit::RADIANS);

  float err = raw_ang_dist.get();
  float err_deg = raw_ang_dist.degrees().get();

  float min = 140, x = fabs(err_deg);
  bool skip = 0;
  if (x == 0) {
    x = 0.00001;
    skip = 1;
  }

  float kp =
      fmax(min, prop_filter[0] + prop_filter[1] * x + prop_filter[2] * 1 / x +
                    prop_filter[3] * pow(x, prop_filter[4]));

  float kd = derivative_filter[0] + derivative_filter[1] * x +
             derivative_filter[2] * x * x + derivative_filter[3] * x * x * x +
             derivative_filter[4] * x * x * x * x;

  float prev = 0, tol = 0.008, tol2 = .007, timeout = 0, timeout2 = 0,
        maxtime = 2;
  float ang, prev_ang = 0;

  // kp = 500;
  // kd = 0;

  while (1) {
    if (skip)
      break;
    targ = desired_heading.radians().get();
    curr = get_heading().get();
    math::Angle raw_ang_dist = math::Angle(
        math::Math::find_min_angle(targ, curr), math::Unit::RADIANS);

    err = raw_ang_dist.get();
    float der = (err - prev);
    prev = err;

    float out = kp * err + kd * der;
    if (direction > 0) {
      move_left(fabs(out) * math::Math::sgn(err));
      move_right(0);
      set_left_brake(utility::BrakeType::COAST);
      set_right_brake(utility::BrakeType::BRAKE);
    } else {
      move_left(0);
      move_right(-fabs(out) * math::Math::sgn(err));
      set_left_brake(utility::BrakeType::BRAKE);
      set_right_brake(utility::BrakeType::COAST);
    }

    ang = curr;
    float del_ang = ang - prev_ang;
    prev_ang = ang;

    if (fabs(err) < tol)
      timeout++;
    else
      timeout = 0;

    if (fabs(del_ang) < tol2)
      timeout2++;
    else
      timeout2 = 0;

    if (timeout >= maxtime || timeout2 >= maxtime + 16)
      break;

    pros::delay(10);
  }
  set_left_brake(utility::BrakeType::COAST);
  set_right_brake(utility::BrakeType::COAST);
}

void Driver::control() {
  float v_out = 0, w_out = 0, dt = 0.01, tolerance = 0.8;
  float accel_static = 1000, vt = 0, vt_prev = 0;
  float accelw_static = 1600, wt = 0, wt_prev = 0;
  float accel, accelw;
  float max_lin = 127;
  float max_ang = 127;
  // float w_deadzone = 85;
  while (1) {
    set_brake(utility::BrakeType::COAST);
    std::uint32_t nw = pros::millis();
    float ve = input::Analog::get_left_y() - v_out;
    float we = input::Analog::get_right_x() - w_out;

    if (math::Math::sgn(ve) < 0)
      accel = accel_static + 300;
    else
      accel = accel_static;
    if (math::Math::sgn(we) < 0)
      accelw = accelw_static + 300;
    else
      accelw = accelw_static;

    if (fabs(ve) > tolerance) {
      vt += math::Math::sgn(ve) * accel * dt;
      vt = math::Math::sgn(vt) * fmin(fabs(vt), max_lin);
      float dv = vt - vt_prev;
      v_out += dv;
      ve -= dv;
      vt_prev = vt;
    }

    if (fabs(we) > tolerance) {
      wt += math::Math::sgn(we) * accelw * dt;
      wt = math::Math::sgn(wt) * fmin(fabs(wt), max_ang);
      float dw = wt - wt_prev;
      w_out += dw;
      we -= dw;
      wt_prev = wt;
    }

    // if (input::Button::down_pressed) {
    // float lvel = input::Analog::get_left_y() + input::Analog::get_right_x();
    // float rvel = input::Analog::get_left_y() - input::Analog::get_right_x();
    // float ratio = std::max(std::abs(lvel), std::abs(rvel)) / 127;
    // if (ratio > 1) {
    //   lvel /= ratio;
    //   rvel /= ratio;
    // }
    // move_left(.6*lvel);
    // move_right(.6*rvel);
    // } else {
    float n = input::Analog::get_right_x();
    float lvel = v_out + n;
    float rvel = v_out - n;
    float ratio = std::max(std::abs(lvel), std::abs(rvel)) / 127;
    if (ratio > 1) {
      lvel /= ratio;
      rvel /= ratio;
    }

    move_left(lvel);
    move_right(rvel);
    // }

    pros::Task::delay_until(&nw, 1000 * dt);
  }
}

void Driver::set_accel_time(float a) { accel_time = a; }

void Driver::set_driver_velocity(float v) { driver_max_velocity = v; }

void Driver::set_controller_tuning(std::string _type, float _curve,
                                   std::string _map, float _accel_time,
                                   float _brake_thresh, float _turn_sens) {
  control_type = _type;
  curve = _curve;
  map_type = _map;
  accel_time = _accel_time;
  brake_thresh = _brake_thresh;
  turn_sens = _turn_sens;
}

float Driver::map(float x) {
  if (x == 0)
    return 0;
  return (127 * pow(std::abs(x), curve)) / pow(127, curve) * (x / std::abs(x));
}

float Driver::logistic_map(float x) {
  if (x < 0)
    return -127 / (1 + exp(-curve * (-x - 63.5)));
  if (x > 0)
    return 127 / (1 + exp(-curve * (x - 63.5)));
  return 0;
}

void none() {}

} // namespace lib::control