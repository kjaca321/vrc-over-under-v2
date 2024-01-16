/**
 * \file pose1d.cpp
 *
 * A Pose1D inherits an x and y from a Vector, as well as all immutable methods
 * from the Vector class. In addition, a Point has a heading, linear velocity,
 * and linear acceleration.
 *
 * @author 3135B
 */

#include "library/api/math/pose1d.hpp"

namespace lib::math {

Pose1D::Pose1D() : pos(0), vel(0), acc(0) {}

Pose1D::Pose1D(float inpos) : pos(inpos), vel(0), acc(0) {}

Pose1D::Pose1D(float inpos, float inv, float ina)
    : pos(inpos), vel(inv), acc(ina) {}

std::string Pose1D::to_string_full() {
  return "pos: " + std::to_string(pos) + ", vel/acc: " + std::to_string(vel) +
         ", " + std::to_string(acc);
}

bool Pose1D::equals_pose(Pose1D rhs) {
  return pos == rhs.pos && vel == rhs.vel && acc == rhs.acc;
}

} // namespace lib::math