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

// void Driver::turn_pt(math::Angle desired_heading) {
//   float targ = desired_heading.radians().get();
//   float curr = get_heading().get();
//   math::Angle raw_ang_dist =
//       math::Angle(math::Math::find_min_angle(targ, curr),
//       math::Unit::RADIANS);
//   float component = raw_ang_dist.get();
//   float dir = math::Math::sgn(component);
//   math::Angle ang_dist = math::Angle(std::abs(component),
//   math::Unit::RADIANS); AngularTrajectory path(ang_dist); float kv = 8, ka =
//   0.05, kp = 2; for (math::Pose1D pose : path.get()) {
//     float des_dist = pose.pos;
//     float omega = pose.vel;
//     float alpha = pose.acc;
//     float left = omega * trackwidth / 2;
//     float right = -omega * trackwidth / 2;
//     float left_out = kv * left + kp * (left - get_left_vel());
//     float right_out = kv * right + kp * (right - get_right_vel());
//     move_left(dir * left_out);
//     move_right(dir * right_out);
//     pros::delay(10);
//   }
// }

void Driver::turn_pt(math::Angle desired_heading, bool rough) {
  float targ = desired_heading.radians().get();
  float curr = get_heading().get();
  float tol = 1, tol2 = 11, prev = 0, integ = 0, int_thresh = 0.5;
  math::Angle raw_ang_dist =
      math::Angle(math::Math::find_min_angle(targ, curr), math::Unit::RADIANS);
  float err = raw_ang_dist.get();
  float err_deg = raw_ang_dist.degrees().get();
  float i = 600, f = 20, p = 0.843, k = 4.62;
  float kp = ((f - i) * std::pow(fabs(err_deg), p)) /
                 (std::pow(fabs(err_deg), p) + std::pow(k, p)) +
             i;
  if(rough) {
    tol += 15;
    tol2 += 15;
    kp += 7;
  }

  while (1) {
    targ = desired_heading.radians().get();
    curr = get_heading().get();
    math::Angle raw_ang_dist = math::Angle(
        math::Math::find_min_angle(targ, curr), math::Unit::RADIANS);
    err = raw_ang_dist.get();

    // float kp = .6, kd = 12, ki = 6;
    // float der = (err - prev);
    // if (err < int_thresh)
    //   integ += err;
    // prev = err;

    // float f_p = kp * func * err;
    // float f_d = kd * der;
    // float f_i = ki * integ;

    // float ratio = (fabs(f_p) + fabs(f_d) + fabs(f_i)) / 127;
    // if (ratio > 1) {
    //   f_p /= ratio;
    //   f_d /= ratio;
    //   f_i /= ratio;
    // }
    // // float outr = f_p + f_d + f_i;
    // float outr = kp * func * err;

    // float out = math::Math::sgn(outr) * fabs(fmax(f, fmin(127, fabs(outr))));
    float out = kp * err;
    move_left(out);
    move_right(-out);
    // pros::lcd::print(2, "out: %f", out);
    // std::cout << "err:" << err << ", func:" << func << ", out:" << out
    //           << std::endl;
    pros::delay(10);
    float cond = (fabs(get_left_vel()) + fabs(get_right_vel()))/2;
    if(rough) {
      if (fabs(out) < tol2)
        break;      
    }
    else {
      if (fabs(out) < tol2 && fabs(err) < tol)
        break;
    }
  }
}

void Driver::control() {
  float v_out = 0, w_out = 0, dt = 0.01, tolerance = 0.8;
  float accel_static = 820, vt = 0, vt_prev = 0;
  float accelw_static = 1500, wt = 0, wt_prev = 0;
  float accel, accelw;
  while (1) {
    set_brake(utility::BrakeType::COAST);
    std::uint32_t nw = pros::millis();
    float ve = input::Analog::get_left_y() - v_out;
    float we = input::Analog::get_right_x() - w_out;

    if (math::Math::sgn(ve) < 0)
      accel = accel_static + 150;
    else
      accel = accel_static;
    if (math::Math::sgn(we) < 0)
      accelw = accelw_static + 150;
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

    if (input::Digital::pressing(input::Button::L2)) {
      float lvel = input::Analog::get_left_y() + input::Analog::get_right_x();
      float rvel = input::Analog::get_left_y() - input::Analog::get_right_x();
      float ratio = std::max(std::abs(lvel), std::abs(rvel)) / 127;
      if (ratio > 1) {
        lvel /= ratio;
        rvel /= ratio;
      }
      move_left(lvel);
      move_right(rvel);
    } else {
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
    }

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