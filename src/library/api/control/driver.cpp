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
  float kv = 1.3, ka = 0.03, kp = .7;
  for (math::Pose1D pose : path.get()) {
    float des_dist = pose.pos;
    float des_vel = pose.vel;
    float des_acc = pose.acc;
    float out = kv * des_vel + ka * des_acc +
                kp * (des_vel - (get_left_vel() + get_right_vel()) / 2);
    move(dir * out);
    pros::delay(10);
  }
}

void Driver::turn_pt(math::Angle desired_heading) {
  float targ = desired_heading.radians().get();
  float curr = get_heading().get();
  math::Angle raw_ang_dist =
      math::Angle(math::Math::find_min_angle(targ, curr), math::Unit::RADIANS);

  float err = raw_ang_dist.get();
  float err_deg = raw_ang_dist.degrees().get();
  float i = 716, f = 60, p = 0.92, k = 2.3;
  float kp = ((f - i) * std::pow(fabs(err_deg), p)) /
                 (std::pow(fabs(err_deg), p) + std::pow(k, p)) +
             i;
  float kd = 400;
  float prev = 0, min = 15, tol = 0.019, tol2 = .007, timeout = 0, timeout2 = 0, maxtime = 2;
  float ang, prev_ang = 0;

  while (1) {
    targ = desired_heading.radians().get();
    curr = get_heading().get();
    math::Angle raw_ang_dist = math::Angle(
        math::Math::find_min_angle(targ, curr), math::Unit::RADIANS);

    err = raw_ang_dist.get();
    float der = (err - prev);
    prev = err;

    float out = kp * err + kd * der;
    if (fabs(out) < min)
      out = math::Math::sgn(out) * min;
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

    if (timeout >= maxtime || timeout2 >= maxtime+8)
      break;
    // if (timeout >= maxtime) break;

    pros::delay(10);
  }
}

void Driver::control() {
  float v_out = 0, w_out = 0, dt = 0.01, tolerance = 0.8;
  float accel_static = 1000, vt = 0, vt_prev = 0;
  float accelw_static = 1600, wt = 0, wt_prev = 0;
  float accel, accelw;
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
      vt = math::Math::sgn(vt) * fmin(fabs(vt), 127);
      float dv = vt - vt_prev;
      v_out += dv;
      ve -= dv;
      vt_prev = vt;
    }

    if (fabs(we) > tolerance) {
      wt += math::Math::sgn(we) * accelw * dt;
      wt = math::Math::sgn(wt) * fmin(fabs(wt), 127);
      float dw = wt - wt_prev;
      w_out += dw;
      we -= dw;
      wt_prev = wt;
    }

    // if (input::Digital::pressing(input::Button::L2)) {
    // float lvel = input::Analog::get_left_y() + input::Analog::get_right_x();
    // float rvel = input::Analog::get_left_y() - input::Analog::get_right_x();
      // float ratio = std::max(std::abs(lvel), std::abs(rvel)) / 127;
      // if (ratio > 1) {
      //   lvel /= ratio;
      //   rvel /= ratio;
      // }
      // move_left(lvel);
      // move_right(rvel);
    // } else {
    float n = input::Analog::get_right_x();
    // float ratio = std::max(std::abs(v_out), std::abs(n)) / 127;
    // if (ratio > 1) {
    //   v_out /= ratio;
    //   n /= ratio;
    // }
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