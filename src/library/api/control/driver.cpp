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
               float wheel, float speed, int imu1)
    : PositionTracker(left_ports, right_ports, wheel, speed, imu1),
      control_type("arcade"), curve(1), map_type("logistic"), accel_time(0.01),
      brake_thresh(0), turn_sens(1), left_y_error(0), left_y_output(0),
      left_x_error(0), left_x_output(0), right_y_error(0), right_y_output(0),
      right_x_error(0), right_x_output(0), left_velocity(0), right_velocity(0),
      left_final_velocity(0), right_final_velocity(0),
      driver_max_velocity(127) {}

void Driver::straight(float distance) {
  Trajectory1D path(distance);
  float kv = 1.3, ka = 0.03, kp = .7;
  for (math::Pose1D pose : path.get()) {
    float des_dist = pose.pos;
    float des_vel = pose.vel;
    float des_acc = pose.acc;
    float out = kv * des_vel + ka * des_acc +
                kp * (des_vel - (get_left_vel() + get_right_vel()) / 2);
    move(out);
    pros::delay(10);
  }
}

void Driver::turn_pt(math::Angle desired_heading) {
  float targ = desired_heading.get();
  float curr = get_heading().get();
  math::Angle ang_dist =
      math::Angle(math::Math::find_min_angle(targ, curr), math::Unit::RADIANS);
  AngularTrajectory path(ang_dist);
  float kv = 7, ka = 0.05, kp = 3;
  for (math::Pose1D pose : path.get()) {
    float des_dist = pose.pos;
    float omega = pose.vel;
    float alpha = pose.acc;
    float out = kv * omega + ka * alpha + kp * (omega - get_angular_vel());
    move_left(out);
    move_right(-out);
    pros::delay(10);
  }
}

void Driver::control() {
  while (1) {
    std::uint32_t nw = pros::millis();

    left_y_error = input::Analog::get_left_y() - left_y_output;
    left_x_error = input::Analog::get_left_x() - left_x_output;
    right_y_error = input::Analog::get_right_y() - right_y_output;
    right_x_error = input::Analog::get_right_x() - right_x_output;

    left_y_output += accel_time * left_y_error;
    left_x_output += accel_time * left_x_error;
    right_y_output += accel_time * right_y_error;
    right_x_output += accel_time * right_x_error;

    if (control_type == "arcade") {
      left_velocity = left_y_output + turn_sens * right_x_output;
      right_velocity = left_y_output - turn_sens * right_x_output;
    } else {
      left_velocity = left_y_output;
      right_velocity = right_y_output;
    }

    if (map_type == "logistic") {
      left_final_velocity = logistic_map(left_velocity);
      right_final_velocity = logistic_map(right_velocity);
    } else {
      left_final_velocity = map(left_velocity);
      right_final_velocity = map(right_velocity);
    }

    if (std::abs(left_final_velocity) > driver_max_velocity)
      left_final_velocity =
          math::Math::sgn(left_final_velocity) * driver_max_velocity;
    if (std::abs(right_final_velocity) > driver_max_velocity)
      right_final_velocity =
          math::Math::sgn(right_final_velocity) * driver_max_velocity;

    if (std::abs(left_final_velocity) < brake_thresh)
      left_final_velocity = 0;
    if (std::abs(right_final_velocity) < brake_thresh)
      right_final_velocity = 0;

    move_left(left_final_velocity);
    move_right(right_final_velocity);

    pros::Task::delay_until(&nw, 5);
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