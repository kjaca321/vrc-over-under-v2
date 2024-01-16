#include "library/api/util/motorsystem.hpp"

namespace lib::utility {

MotorSystem::MotorSystem(std::vector<int> left_ports,
                         std::vector<int> right_ports)
    : Chassis(left_ports, right_ports) {}

float MotorSystem::get_raw_left_pos() {
  float sum = 0;
  for (pros::Motor *i : left_motors)
    sum += i->get_position();
  return sum / 3;
}

float MotorSystem::get_raw_right_pos() {
  float sum = 0;
  for (pros::Motor *i : right_motors)
    sum += i->get_position();
  return sum / 3;
}

void MotorSystem::reset_left_side() {
  for (pros::Motor *i : left_motors)
    i->tare_position();
}

void MotorSystem::reset_right_side() {
  for (pros::Motor *i : right_motors)
    i->tare_position();
}

float MotorSystem::get() {
  return (get_raw_left_pos() + get_raw_right_pos()) / 2.0;
}

}; // namespace lib::utility