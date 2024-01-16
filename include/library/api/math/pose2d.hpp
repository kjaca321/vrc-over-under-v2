/**
 * \file pose2d.hpp
 *
 * A Pose2D inherits an x and y from a Vector, as well as all immutable methods
 * from the Vector class. In addition, a Point has a heading, linear velocity,
 * linear acceleration, and angular velocity.
 *
 * @author 3135B
 */

#pragma once
#include "angle.hpp"
#include "vector.hpp"

namespace lib::math {

class Pose2D : public Vector {
public:
  Pose2D(void);
  Pose2D(float inx, float iny);
  Pose2D(float inx, float iny, Angle inheading);
  Pose2D(float inx, float iny, Angle inheading, float inv, float ina,
         float inw);

  std::string to_string_full(void);
  bool equals_pose(Pose2D rhs);

  Angle heading;
  float linear_vel;
  float linear_acc;
  float angular_vel;
};

} // namespace lib::math