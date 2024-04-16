/**
 * \file trajectory1d.cpp
 *
 * Trajectory1D class, creates and configures an array of robot velocities.
 *
 * @author 3135B
 */

#include "library/api/control/trajectory1d.hpp"

namespace lib::control {

void Trajectory1D::set_constraints(float in_vel, float in_accel,
                                   float in_min_vel) {
  global_max_velocity = in_vel;
  global_max_acceleration = in_accel;
  global_min_velocity = in_min_vel;
}

Trajectory1D::Trajectory1D(float desired_dist) {
  total_distance = desired_dist;
  acc = global_max_acceleration, vel0 = global_min_velocity,
  vels = global_max_velocity, dist = total_distance;
  float initial_factor = (dist * acc + vel0 * vel0) / (2 * vels) + vels / 2;
  float factor =
      (dist > 25) ? (initial_factor - 5)
                  : ((dist > 10) ? (initial_factor - 10) : initial_factor - 15);
  internal_max_vel = std::min(vels, factor);
  vel = internal_max_vel;

  float dist_trav = 0;
  float time = 0;
  trajectory = {};
  while (dist_trav <= total_distance) {
    math::Pose1D kinematics = get_kinematics(dist_trav);
    trajectory.push_back(kinematics);
    dist_trav += kinematics.vel * dt;
    time += dt;
  }
}

std::vector<math::Pose1D> Trajectory1D::get() { return trajectory; }

math::Pose1D Trajectory1D::get_kinematics(float distance_travelled) {
  /* kinematic equations for timestamps of acceleration change derived from the
   * constant-acceleration formulas */
  float t1 = (vel - vel0) / acc;
  float t2 = dist / vel - vel0 / acc + (vel0 * vel0) / (acc * vel);
  float tf =
      dist / vel + vel / acc - 2 * vel0 / acc + (vel0 * vel0) / (acc * vel);

  /* discrete integrals of velocity at timestamps t1 and t2 (accumulated
   * distance/arc length) */
  float d1 = acc / 2 * t1 * t1 + vel0 * t1;
  float d2 = d1 + vel * t2 - vel * t1;

  /* based on current distance travelled, return desired linear acceleration and
   * velocity derived from the kinematic timestamp equations */
  
  if (distance_travelled < d1) // first segment (positive acceleration)
    return math::Pose1D(
        distance_travelled,
        (vel0 + std::sqrt(vel0 * vel0 + 8 * distance_travelled * acc)) / 2,
        acc);
  
  if (distance_travelled < d2) // second segment (constant velocity)
    return math::Pose1D(distance_travelled, vel, 0);
  
  else // third segment (negative acceleration)
    return math::Pose1D(
        distance_travelled,
        (vel + acc * t2 - acc * tf +
         std::sqrt(acc * acc * t2 * t2 + acc * acc * tf * tf + vel * vel -
                   2 * acc * acc * t2 * tf + 2 * acc * vel * t2 -
                   2 * acc * vel * tf - 8 * acc * distance_travelled +
                   8 * acc * dist)) /
            2,
        -acc);
}

}; // namespace lib::control
