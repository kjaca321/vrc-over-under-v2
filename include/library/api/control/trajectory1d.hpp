/**
 * \file trajectory1d.hpp
 *
 * Trajectory1D class, creates and configures an array of robot velocities.
 *
 * @author 3135B
 */

#pragma once
#include "../../util.hpp"
#include "../math/angle.hpp"
#include "../math/math.hpp"
#include "../math/point.hpp"
#include "../math/pose1d.hpp"
#include "../math/vector.hpp"

namespace lib::control {

class Trajectory1D {
public:
  Trajectory1D(float desired_dist);
  Trajectory1D(float desired_dist, std::vector<math::Vector> speeds);
  static void set_constraints(float in_vel, float in_accel, float in_min_vel);
  std::vector<math::Pose1D> get(void);

private:
  math::Pose1D get_kinematics(float distance_travelled);
  std::vector<math::Pose1D> trajectory;
  float total_distance;
  float internal_max_vel;
  static inline float global_max_velocity = 0;
  static inline float global_max_acceleration = 0;
  static inline float global_min_velocity = 0;
  float acc, vel, vel0, vels, dist;
  static inline float dt = 0.01;
};

} // namespace lib::control