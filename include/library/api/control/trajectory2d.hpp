/**
 * \file trajectory2d.hpp
 *
 * Trajectory2D class, creates and configures an array of robot velocities.
 *
 * @author 3135B
 */

#pragma once
#include "../../util.hpp"
#include "../math/angle.hpp"
#include "../math/math.hpp"
#include "../math/point.hpp"
#include "../math/pose2d.hpp"
#include "../math/vector.hpp"

namespace lib::control {

class Trajectory2D {
public:
  Trajectory2D(std::vector<math::Vector> raw_path, float c = 2, float b = 0.82,
               float k = 7);
  Trajectory2D(std::vector<math::Vector> raw_path,
               std::vector<math::Vector> speeds, float c = 2, float b = 0.82,
               float k = 7);
  static void set_constraints(float in_vel, float in_accel, float in_min_vel);
  static void set_constraints(float in_vel, float in_accel, float in_min_vel,
                              float in_trackw);
  std::vector<math::Pose2D> get(void);
  std::vector<math::Point> get_path(void);

private:
  std::vector<math::Vector> interpolate(math::Vector a, math::Vector b);
  std::vector<math::Vector> smoothen(std::vector<math::Vector> p);
  std::vector<math::Vector>
  generate_descent(std::vector<math::Vector> raw_path);
  static float get_point_curvature(math::Vector prev, math::Vector curr,
                                   math::Vector next);
  std::vector<math::Point> generate_path(std::vector<math::Vector> spline);
  static math::Angle get_heading_at_pt(std::vector<math::Vector> spline,
                                       int idx);
  math::Vector get_profiled_linear_kinematics(float distance_travelled);
  math::Point get_path_parameters(float distance_travelled);
  math::Pose2D get_kinematics(float distance_travelled);
  std::vector<math::Pose2D> trajectory;
  std::vector<math::Point> path;
  float total_distance;
  float spacing;
  float dampener;
  float arc_constant;
  float internal_max_vel;
  static inline float global_max_velocity = 0;
  static inline float global_max_acceleration = 0;
  static inline float global_min_velocity = 0;
  static inline float global_trackwidth = 0;
  float acc, vel, vel0, vels, dist;
  static inline float dt = 0.01;
};

} // namespace lib::control