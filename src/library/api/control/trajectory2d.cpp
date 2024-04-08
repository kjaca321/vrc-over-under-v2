/**
 * \file trajectory.cpp
 *
 * Trajectory2D class, creates and configures an array of robot velocities.
 *
 * @author 3135B
 */

#include "library/api/control/trajectory2d.hpp"

namespace lib::control {

void Trajectory2D::set_constraints(float in_vel, float in_accel,
                                   float in_min_vel) {
  global_max_velocity = in_vel;
  global_max_acceleration = in_accel;
  global_min_velocity = in_min_vel;
}

void Trajectory2D::set_constraints(float in_vel, float in_accel,
                                   float in_min_vel, float in_trackw) {
  global_max_velocity = in_vel;
  global_max_acceleration = in_accel;
  global_min_velocity = in_min_vel;
  global_trackwidth = in_trackw;
}

Trajectory2D::Trajectory2D(math::CubicBezier raw_path, float c, float b,
                           float k)
    : spacing(c), dampener(b), arc_constant(k) {

  total_distance = 0;
  std::vector<math::Vector> spline = {};

  float density = 0.02; // spacing of points along the curve on (0, 1)
  std::vector<float> lengths = {};

  // for each point on (0 + density, 1)
  for (float t = density; t <= 1; t += density) {

    // retrieve distance between previous and current locations on the curve
    math::Vector prev = raw_path.get_raw(t - density);
    math::Vector curr = raw_path.get_raw(t);
    float d = curr.distance(prev);

    // accumulate distance to a running total and an array of partial sums
    lengths.push_back(total_distance);
    total_distance += d;
  }

  // compute number of points as the reciprocal of the point density
  int num_pts = (int)(1 / density);

  float map_dist;

  // iterate through all points on the curve on (0, 1)
  for (float u = 0; u <= 1; u += density) {
    /* compute arc length of the curve at iterator timestamp -> location on
     * (0,1) multiplied by the total arc length */
    float targ_length = u * lengths[num_pts - 1];

    /* perform a binary search for the closest possible match in the array of
     * partial distance sums to the desired arc length calculated above */
    int low = 0, high = num_pts, idx = 0;
    while (low < high) {
      idx = low + (((high - low) / 2) | 0);
      if (lengths[idx] < targ_length) {
        low = idx + 1;

      } else {
        high = idx;
      }
    }
    if (lengths[idx] > targ_length) {
      idx--;
    }

    // retrieve partial sum at index found by binary search
    float prev_length = lengths[idx];

    // in the case of an exact match to the target arc length
    if (prev_length == targ_length) {
      // assign a new t value on (0,1) based on the exact match index
      map_dist = (float)idx / num_pts;
    }

    // without an exact match
    else {
      /* create a fractional index from change in distance from closest match to
       * target arc length, and assign a new t value on (0,1) based on the
       * fractional index */
      map_dist = (idx + (targ_length - prev_length) /
                            (lengths[idx + 1] - prev_length)) /
                 num_pts;
    }

    /* append a math::Vector containing the x and y for each updated timestamp
     * to a new array of Vectors */
    spline.push_back((raw_path.get_raw(map_dist)));
  }

  acc = global_max_acceleration, dist = total_distance,
  vels = global_max_velocity, vel0 = global_min_velocity;

  float initial_factor = (dist * acc + vel0 * vel0) / (2 * vels) + vels / 2;

  // conditional ladder to limit the velocity based on total target distance
  float factor =
      (dist > 25) ? (initial_factor - 5)
                  : ((dist > 10) ? (initial_factor - 10) : initial_factor - 15);
  internal_max_vel = std::min(vels, factor);
  vel = internal_max_vel;

  path = generate_path(spline);

  // intialize iterators and resulting storage
  float dist_trav = 0;
  float time = 0;
  trajectory = {};

  // iterating through the arc length of the curve
  while (dist_trav <= dist) {
    /*retrieve the linear velocity and curvature of the path at the current
     * arc length */
    math::Pose2D kinematics = get_kinematics(dist_trav);
    float lin = kinematics.linear_vel, curv = kinematics.angular_vel;
    float current_acc = kinematics.linear_acc;

    float c1 = raw_path.get_curvature(dist_trav / dist);
    // if (c1 < 0.000001)
    // curv = math::Math::sgn(curv) * c1;
    // else
    //   curv = math::Math::sgn(curv) * c1;

    /* convert linear velocity and curvature to left and right wheel velocities
     * (from the kinematics of a differential drive robot) */
    float lvel = lin * (2 + curv * global_trackwidth) / 2;
    float rvel = lin * (2 - curv * global_trackwidth) / 2;

    /* to prevent saturation of motor velocities, preserve ratio bewtween
     * left and right sides if max velocity is exceeded */
    float ratio = std::max(std::abs(lvel), std::abs(rvel)) / vels;
    if (ratio > 1) {
      lvel /= ratio;
      rvel /= ratio;
    }

    /* convert left and right wheel velocities to linear and angular velocities
     * (from the kinematics of a differential drive robot) */
    float final_lin = (lvel + rvel) / 2;
    float final_ang = (lvel - rvel) / global_trackwidth;

    // update resulting trajectory and arc length / time iterators
    trajectory.push_back(math::Pose2D(kinematics.x, kinematics.y,
                                      kinematics.heading, final_lin,
                                      current_acc, final_ang));

    dist_trav += final_lin * dt;
    time += dt;
  }
}

Trajectory2D::Trajectory2D(math::CubicBezier raw_path,
                           std::vector<math::Vector> speeds, float c, float b,
                           float k) {

  total_distance = 0;
  std::vector<math::Vector> spline = {};

  float density = 0.005;
  for (float t = density; t <= 1; t += density) {
    math::Vector prev = raw_path.get_raw(t - density);
    math::Vector curr = raw_path.get_raw(t);
    float d = curr.distance(prev);
    total_distance += d;
    spline.push_back(prev);
  }
  spline.push_back(raw_path.get_raw(1));

  // for (int i = 1; i < spline.size(); i++)
  //   total_distance += spline[i].distance(spline[i - 1]);

  acc = global_max_acceleration, dist = total_distance,
  vels = global_max_velocity, vel0 = global_min_velocity;

  float initial_factor = (dist * acc + vel0 * vel0) / (2 * vels) + vels / 2;
  float factor =
      (dist > 25) ? (initial_factor - 5)
                  : ((dist > 10) ? (initial_factor - 10) : initial_factor - 15);

  std::vector<math::Vector> compounded_speeds = {};
  float compounded_dist = 0;
  for (math::Vector i : speeds) {
    compounded_dist += i.x;
    compounded_speeds.push_back(math::Vector(compounded_dist, i.y));
  }

  internal_max_vel = std::min(vels, factor);
  vel = internal_max_vel;

  path = generate_path(spline);

  float dist_trav = 0;
  float time = 0;
  trajectory = {};
  while (dist_trav <= dist) {
    for (math::Vector i : compounded_speeds)
      if (dist_trav <= i.x) {
        internal_max_vel = std::min({vels, i.y, factor});
        break;
      }
    vel = internal_max_vel;
    math::Pose2D kinematics = get_kinematics(dist_trav);
    float lin = kinematics.linear_vel, curv = kinematics.angular_vel, tr = 11;
    float current_acc = kinematics.linear_acc;
    float lvel = lin * (2 + curv * tr) / 2;
    float rvel = lin * (2 - curv * tr) / 2;
    float ratio = std::max(std::abs(lvel), std::abs(rvel)) / vels;
    if (ratio > 1) {
      lvel /= ratio;
      rvel /= ratio;
    }
    float final_lin = (lvel + rvel) / 2;
    float final_ang = (lvel - rvel) / tr;
    trajectory.push_back(math::Pose2D(kinematics.x, kinematics.y,
                                      kinematics.heading, final_lin,
                                      current_acc, final_ang));
    dist_trav += final_lin * dt;
    time += dt;
  }
}

std::vector<math::Pose2D> Trajectory2D::get() { return trajectory; }

std::vector<math::Point> Trajectory2D::get_path() { return path; }

std::vector<math::Vector> Trajectory2D::interpolate(math::Vector a,
                                                    math::Vector b) {
  math::Vector v = b - a;
  int n = ceil(v.magnitude() / spacing);
  v = v.normalize() * spacing;
  std::vector<math::Vector> res;
  for (int i = 0; i < n; i++)
    res.push_back(a + v * i);
  return res;
}

std::vector<math::Vector> Trajectory2D::smoothen(std::vector<math::Vector> p) {
  std::vector<math::Vector> new_p = p;
  float aux;
  float tol = 0.001;
  float a = 1 - dampener;
  float chg = tol;
  while (chg >= tol) {
    chg = 0;
    for (int i = 1; i < p.size() - 1; i++) {
      aux = new_p[i].x;
      new_p[i].x +=
          a * (p[i].x - new_p[i].x) +
          dampener * (new_p[i - 1].x + new_p[i + 1].x - (2.0 * new_p[i].x));
      chg += std::abs(aux - new_p[i].x);
      aux = new_p[i].y;
      new_p[i].y +=
          a * (p[i].y - new_p[i].y) +
          dampener * (new_p[i - 1].y + new_p[i + 1].y - (2.0 * new_p[i].y));
      chg += std::abs(aux - new_p[i].y);
    }
  }
  return new_p;
}

std::vector<math::Vector>
Trajectory2D::generate_descent(std::vector<math::Vector> raw_path) {
  std::vector<math::Vector> spliced = {};
  for (int i = 0; i < raw_path.size() - 1; i++) {
    std::vector<math::Vector> tmp = interpolate(raw_path[i], raw_path[i + 1]);
    spliced.insert(spliced.end(), tmp.begin(), tmp.end());
  }
  spliced.push_back(raw_path.back());
  return smoothen(spliced);
}

float Trajectory2D::get_point_curvature(math::Vector prev, math::Vector curr,
                                        math::Vector next) {
  float A = prev.x * (curr.y - next.y) - prev.y * (curr.x - next.x) +
            curr.x * next.y - next.x * curr.y;
  if (A == 0)
    return 0;

  float B = (prev.x * prev.x + prev.y * prev.y) * (next.y - curr.y) +
            (curr.x * curr.x + curr.y * curr.y) * (prev.y - next.y) +
            (next.x * next.x + next.y * next.y) * (curr.y - prev.y);

  float C = (prev.x * prev.x + prev.y * prev.y) * (curr.x - next.x) +
            (curr.x * curr.x + curr.y * curr.y) * (next.x - prev.x) +
            (next.x * next.x + next.y * next.y) * (prev.x - curr.x);

  float D =
      (prev.x * prev.x + prev.y * prev.y) *
          (next.x * curr.y - curr.x * next.y) +
      (curr.x * curr.x + curr.y * curr.y) *
          (prev.x * next.y - next.x * prev.y) +
      (next.x * next.x + next.y * next.y) * (curr.x * prev.y - prev.x * curr.y);

  float x0 = -B / (2 * A);
  float y0 = -C / (2 * A);
  float r = sqrt((B * B + C * C - 4 * A * D) / (4 * A * A));

  float sign = 0;
  bool trend = prev.y <= curr.y && curr.y <= next.y; // 1 if up, 0 if down
  if (trend) {
    if (y0 < curr.y && x0 > curr.x)
      sign = 1; // L
    if (y0 > curr.y && x0 < curr.x)
      sign = -1; // R
    if (y0 > curr.y && x0 > curr.x)
      sign = 1; // L
    if (y0 < curr.y && x0 < curr.x)
      sign = -1; // R
  } else {
    if (y0 < curr.y && x0 > curr.x)
      sign = -1; // R
    if (y0 > curr.y && x0 < curr.x)
      sign = 1; // L
    if (y0 > curr.y && x0 > curr.x)
      sign = -1; // R
    if (y0 < curr.y && x0 < curr.x)
      sign = 1; // L
  }
  return 1 / r * sign;
}

std::vector<math::Point>
Trajectory2D::generate_path(std::vector<math::Vector> spline) {
  std::vector<math::Point> res = {};
  for (int i = 0; i < spline.size(); i++) {
    math::Angle curr_heading = get_heading_at_pt(spline, i);
    float curvature;
    if (i == 0 or i == spline.size() - 1)
      curvature = 0;
    else
      curvature = get_point_curvature(spline[i - 1], spline[i], spline[i + 1]);
    float velocity =
        std::min(internal_max_vel, arc_constant / std::abs(curvature));
    res.push_back(math::Point(spline[i].x, spline[i].y, curr_heading.radians(),
                              velocity, curvature));
  }
  return res;
}

math::Angle Trajectory2D::get_heading_at_pt(std::vector<math::Vector> spline,
                                            int idx) {
  // recursive edge case: heading at first point is set to heading at second
  if (idx == 0)
    return get_heading_at_pt(spline, idx + 1);

  // retrieve discrete derivative of the curve at the desired point from slope
  float run = spline[idx].x - spline[idx - 1].x;
  float rise = spline[idx].y - spline[idx - 1].y;

  // angle wrapping: declare conditions for creating absolute heading
  bool up = spline[idx - 1].y < spline[idx].y;
  bool right = spline[idx - 1].x < spline[idx].x;

  math::Angle relative;

  // determine reference angle heading with the arctan function
  if (rise == 0)
    relative = math::Angle(M_PI_2, math::Unit::RADIANS);
  else
    relative = math::Angle(atan(run / rise), math::Unit::RADIANS);

  // wrap angle to absolute based on direction of curve
  if (spline[idx - 1].y == spline[idx].y) {
    if (!right)
      return math::Angle((M_PI - std::abs(relative.get())) *
                             -math::Math::sgn(relative.get()),
                         math::Unit::RADIANS);
    return relative;
  } else {
    if (!up)
      return math::Angle((M_PI - std::abs(relative.get())) *
                             -math::Math::sgn(relative.get()),
                         math::Unit::RADIANS);
    return relative;
  }
}

math::Vector
Trajectory2D::get_profiled_linear_kinematics(float distance_travelled) {
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
    return math::Vector(
        (vel0 + std::sqrt(vel0 * vel0 + 8 * distance_travelled * acc)) / 2,
        acc);

  if (distance_travelled < d2) // second segment (constant velocity)
    return math::Vector(vel, 0);

  else // third segment (negative acceleration)
    return math::Vector(
        (vel + acc * t2 - acc * tf +
         std::sqrt(acc * acc * t2 * t2 + acc * acc * tf * tf + vel * vel -
                   2 * acc * acc * t2 * tf + 2 * acc * vel * t2 -
                   2 * acc * vel * tf - 8 * acc * distance_travelled +
                   8 * acc * dist)) /
            2,
        -acc);
}

math::Point Trajectory2D::get_path_parameters(float distance_travelled) {
  int idx =
      (int)round((distance_travelled / total_distance) * (path.size() - 1));
  return path[idx];
}

math::Pose2D Trajectory2D::get_kinematics(float distance_travelled) {
  math::Vector grab = get_profiled_linear_kinematics(distance_travelled);
  float profiled_vel = grab.x, profiled_acc = grab.y;
  math::Point path_param = get_path_parameters(distance_travelled);
  float lin_vel = std::min(profiled_vel, path_param.linear_velocity);
  return math::Pose2D(path_param.x, path_param.y, path_param.heading, lin_vel,
                      profiled_acc, path_param.curvature);
}

} // namespace lib::control