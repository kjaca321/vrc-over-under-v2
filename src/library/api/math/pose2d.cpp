/**
 * \file pose2d.cpp
 *
 * A Pose2D inherits an x and y from a Vector, as well as all immutable methods
 * from the Vector class. In addition, a Point has a heading, linear velocity,
 * linear acceleration, and angular velocity.
 *
 * @author 3135B
 */

#include "library/api/math/pose2d.hpp"

namespace lib::math {

Pose2D::Pose2D()
    : Vector(0, 0), heading(Angle()), linear_vel(0), linear_acc(0),
      angular_vel(0) {}

Pose2D::Pose2D(float inx, float iny)
    : Vector(inx, iny), heading(Angle()), linear_vel(0), linear_acc(0),
      angular_vel(0) {}

Pose2D::Pose2D(float inx, float iny, Angle inheading, float inv, float ina,
               float inw)
    : Vector(inx, iny), heading(inheading), linear_vel(inv), linear_acc(ina),
      angular_vel(inw) {}

std::string Pose2D::to_string_full() {
  return "pos: " + to_string() + ", heading: " + heading.to_string() +
         ", linear vel/acc: " + std::to_string(linear_vel) + ", " +
         std::to_string(linear_acc) +
         ", angular vel: " + std::to_string(angular_vel);
}

bool Pose2D::equals_pose(Pose2D rhs) {
  return equals(rhs) && heading.equals(rhs.heading) &&
         linear_vel == rhs.linear_vel && linear_acc == rhs.linear_acc &&
         angular_vel == rhs.angular_vel;
}

} // namespace lib::math