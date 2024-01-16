/**
 * \file angulartrajectory.hpp
 *
 * AngularTrajectory class, creates and configures an array of robot velocities.
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

class AngularTrajectory {
public:
  AngularTrajectory(math::Angle desired_delta_heading);
  static void set_constraints(float in_vel, float in_accel, float in_min_vel);
  static void set_constraints(float in_vel, float in_accel, float in_min_vel,
                              float in_trackw);
  std::vector<math::Pose1D> get(void);

private:
  math::Pose1D get_kinematics(float angle_travelled);
  std::vector<math::Pose1D> trajectory;
  float total_angle;
  float internal_max_vel;
  static inline float global_max_velocity = 0;
  static inline float global_max_acceleration = 0;
  static inline float global_min_velocity = 0;
  static inline float global_trackwidth = 0;
  float ang_acc, ang_vel, ang_vel0, ang_vels, ang_dist;
  static inline float dt = 0.01;
};

} // namespace lib::control