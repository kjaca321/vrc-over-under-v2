/**
 * \file pose1d.hpp
 *
 * A Pose1D inherits an x and y from a Vector, as well as all immutable methods
 * from the Vector class. In addition, a Point has a heading, linear velocity,
 * and linear acceleration.
 *
 * @author 3135B
 */

#pragma once
#include "angle.hpp"
#include "vector.hpp"

namespace lib::math {

class Pose1D {
public:
  Pose1D(void);
  Pose1D(float inpos);
  Pose1D(float inpos, float inv, float ina);

  std::string to_string_full(void);
  bool equals_pose(Pose1D rhs);

  float pos;
  float vel;
  float acc;
};

} // namespace lib::math