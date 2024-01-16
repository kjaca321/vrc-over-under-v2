/**
 * \file angulartrajectory.cpp
 *
 * AngularTrajectory class, creates and configures an array of robot velocities.
 *
 * @author 3135B
 */

#include "library/api/control/angulartrajectory.hpp"

namespace lib::control {

void AngularTrajectory::set_constraints(float in_vel, float in_accel,
                                        float in_min_vel) {
  global_max_velocity = in_vel;
  global_max_acceleration = in_accel;
  global_min_velocity = in_min_vel;
}

AngularTrajectory::AngularTrajectory(math::Angle desired_delta_heading) {
  total_angle = desired_delta_heading.radians().get();
  ang_acc = global_max_acceleration, ang_vel0 = global_min_velocity,
  ang_vels = global_max_velocity, ang_dist = total_angle;
  float initial_factor =
      (ang_dist * ang_acc + ang_vel0 * ang_vel0) / (2 * ang_vels) +
      ang_vels / 2;
  float factor;
  if (ang_dist > 3.75)
    factor = initial_factor - 1;
  else if (ang_dist > 2)
    factor = initial_factor - 1.5;
  else if (ang_dist > 1)
    factor = initial_factor - 2;
  else if (ang_dist > .5)
    factor = initial_factor - 2.75;
  else if (ang_dist > .25)
    factor = initial_factor - 3.5;
  else
    factor = initial_factor - 4;

  internal_max_vel = std::min(ang_vels, factor);
  ang_vel = internal_max_vel;

  float ang_trav = 0;
  float time = 0;
  trajectory = {};
  while (ang_trav <= total_angle) {
    math::Pose1D kinematics = get_kinematics(ang_trav);
    trajectory.push_back(kinematics);
    ang_trav += kinematics.vel * dt;
    time += dt;
  }
}

std::vector<math::Pose1D> AngularTrajectory::get() { return trajectory; }

math::Pose1D AngularTrajectory::get_kinematics(float angle_travelled) {
  float t1 = (ang_vel - ang_vel0) / ang_acc;
  float t2 = ang_dist / ang_vel - ang_vel0 / ang_acc +
             (ang_vel0 * ang_vel0) / (ang_acc * ang_vel);
  float tf = ang_dist / ang_vel + ang_vel / ang_acc - 2 * ang_vel0 / ang_acc +
             (ang_vel0 * ang_vel0) / (ang_acc * ang_vel);
  float d1 = ang_acc / 2 * t1 * t1 + ang_vel0 * t1;
  float d2 = d1 + ang_vel * t2 - ang_vel * t1;
  if (angle_travelled < d1)
    return math::Pose1D(angle_travelled,
                        (ang_vel0 + std::sqrt(ang_vel0 * ang_vel0 +
                                              8 * angle_travelled * ang_acc)) /
                            2,
                        ang_acc);
  if (angle_travelled < d2)
    return math::Pose1D(angle_travelled, ang_vel, 0);
  else
    return math::Pose1D(
        angle_travelled,
        (ang_vel + ang_acc * t2 - ang_acc * tf +
         std::sqrt(ang_acc * ang_acc * t2 * t2 + ang_acc * ang_acc * tf * tf +
                   ang_vel * ang_vel - 2 * ang_acc * ang_acc * t2 * tf +
                   2 * ang_acc * ang_vel * t2 - 2 * ang_acc * ang_vel * tf -
                   8 * ang_acc * angle_travelled + 8 * ang_acc * ang_dist)) /
            2,
        -ang_acc);
}

} // namespace lib::control