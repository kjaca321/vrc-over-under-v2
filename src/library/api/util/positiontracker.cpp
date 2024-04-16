/**
 * \file positiontracker.cpp
 *
 * PositionTracker class, contains static methods to track the position of the
 * robot.
 *
 * @author 3135B
 */

#include "library/api/util/positiontracker.hpp"

namespace lib::utility {

PositionTracker::PositionTracker(std::vector<int> left_ports,
                                 std::vector<int> right_ports, float wheel,
                                 float speed, int imu1)
    : previous(0), prev_heading(0), rel_prev_heading(0), prev_left(0),
      prev_right(0), prev_left_vel(0), prev_right_vel(0),
      relative_distance(0), control::Chassis(left_ports, right_ports),
      wheel_size(wheel), rpm(speed), odom_dt(0.01) {
  position = math::Vector(0, 0);
  relative_position = math::Vector(0, 0);
  imu = new pros::Imu(imu1);
  imu_remap = new pros::Imu(imu1);
}

float PositionTracker::get_raw_left_pos() {
  float sum = 0;
  for (pros::Motor *i : left_motors)
    sum += i->get_position();
  return sum / 3;
}

float PositionTracker::get_raw_right_pos() {
  float sum = 0;
  for (pros::Motor *i : right_motors)
    sum += i->get_position();
  return sum / 3;
}

void PositionTracker::reset_left_side() {
  for (pros::Motor *i : left_motors)
    i->tare_position();
}

void PositionTracker::reset_right_side() {
  for (pros::Motor *i : right_motors)
    i->tare_position();
}

void PositionTracker::run_tracker() {
  while (1) {
    std::uint32_t nw = pros::millis();

    math::Angle curr_heading =
        math::Angle(fmod((imu->get_heading() + 180.0), 360.0) - 180.0,
                    math::Unit::DEGREES)
            .radians();
    math::Angle delta = math::Angle(curr_heading.get() - prev_heading_i.get(),
                                    math::Unit::RADIANS);

    angular_vel = delta.get() / odom_dt;
    angular_acc = (angular_vel - prev_angular_vel) / odom_dt;
    prev_angular_vel = angular_vel;

    heading = math::Angle(heading.get() + delta.get(), math::Unit::RADIANS);
    relative_heading =
        math::Angle(relative_heading.get() + delta.get(), math::Unit::RADIANS);
    prev_heading_i = math::Angle(curr_heading.get(), math::Unit::RADIANS);

    float pos = (get_raw_left_pos() + get_raw_right_pos()) / 2.0, d_pos_local,
          rel_d_pos_local;
    float speed = M_PI * wheel_size * (rpm / 600.0) * (pos - previous) / 360.0;

    float left_pos = get_raw_left_pos();
    float left_speed =
        M_PI * wheel_size * (rpm / 600.0) * (left_pos - prev_left) / 360.0;
    prev_left = left_pos;

    float right_pos = get_raw_right_pos();
    float right_speed =
        M_PI * wheel_size * (rpm / 600.0) * (right_pos - prev_right) / 360.0;
    prev_right = right_pos;

    odom_left_vel = left_speed / odom_dt;
    odom_right_vel = right_speed / odom_dt;

    odom_left_acc = (odom_left_vel - prev_left_vel) / odom_dt;
    prev_left_vel = odom_left_vel;

    odom_right_acc = (odom_right_vel - prev_right_vel) / odom_dt;
    prev_right_vel = odom_right_vel;

    previous = pos;
    float ang = heading.get();
    float d_heading = ang - prev_heading;
    float partial = d_heading / 2;
    float avg = ang + partial;
    prev_heading = ang;

    float rel_ang = relative_heading.get();
    float rel_d_heading = rel_ang - rel_prev_heading;
    float rel_partial = d_heading / 2;
    float rel_avg = rel_ang + rel_partial;
    rel_prev_heading = rel_ang;

    if (d_heading == 0)
      d_pos_local = speed;
    else
      d_pos_local = 2 * sin(d_heading / 2) * (speed / d_heading);

    if (rel_d_heading == 0)
      rel_d_pos_local = speed;
    else
      rel_d_pos_local = 2 * sin(rel_d_heading / 2) * (speed / rel_d_heading);

    position.x += d_pos_local * sin(avg);
    position.y += d_pos_local * cos(avg);

    relative_position.x += rel_d_pos_local * sin(rel_avg);
    relative_position.y += rel_d_pos_local * cos(rel_avg);

    // relative_position.y += speed * cos(heading.get());
    // relative_position.x += speed * sin(heading.get());

    relative_distance += speed;

    pros::lcd::print(0, "%s", relative_position.to_string());
    pros::lcd::print(1, "currheading: %f", curr_heading.degrees().get());

    pros::Task::delay_until(&nw, odom_dt * 1000);
  }
}

void PositionTracker::set_position(math::Vector pos, math::Angle heading) {
  position = pos;
  set_heading(heading);
}

math::Vector PositionTracker::get_position() { return position; }

void PositionTracker::set_heading(math::Angle a) { heading = a.radians(); }

math::Angle PositionTracker::get_heading() { return heading; }

void PositionTracker::set_relative_heading(math::Angle a) {
  relative_heading = a;
}

math::Angle PositionTracker::get_relative_heading() { return relative_heading; }

float PositionTracker::get_relative_displacement() { return relative_distance; }

void PositionTracker::set_relative_displacement(float distance) {
  relative_distance = distance;
}

math::Vector PositionTracker::get_relative_position() {
  return relative_position;
}

void PositionTracker::set_relative_position(math::Vector pos) {
  relative_position = pos;
}

float PositionTracker::get_left_vel() { return odom_left_vel; }

float PositionTracker::get_right_vel() { return odom_right_vel; }

float PositionTracker::get_left_acc() { return odom_left_acc; }

float PositionTracker::get_right_acc() { return odom_right_acc; }

float PositionTracker::get_angular_vel() { return angular_vel; }
float PositionTracker::get_angular_acc() { return angular_acc; }

void PositionTracker::setup() {
  imu->reset(1);
  reset_left_side();
  reset_right_side();
  set_position(math::Vector(0, 0), math::Angle(0, math::Unit::DEGREES));
}

}; // namespace lib::utility