/**
 * \file driver.cpp
 *
 * Driver class, controls all chassis movements, both autonomous and controlled.
 *
 * @author 3135B
 */

#include "library/api/control/driver.hpp"

namespace lib::control {

Driver::Driver(std::vector<int> left_ports, std::vector<int> right_ports,
               float wheel, float speed, int imu1, float trackw)
    : PositionTracker(left_ports, right_ports, wheel, speed, imu1),
      control_type("arcade"), curve(1), map_type("logistic"), accel_time(0.01),
      brake_thresh(0), trackwidth(trackw), turn_sens(1), left_y_error(0),
      left_y_output(0), left_x_error(0), left_x_output(0), right_y_error(0),
      right_y_output(0), right_x_error(0), right_x_output(0), left_velocity(0),
      right_velocity(0), left_final_velocity(0), right_final_velocity(0),
      driver_max_velocity(127) {}

void Driver::straight(float distance) {
  // seperate sign from magnitude of target value (for feed control)
  float dir = math::Math::sgn(distance);
  distance = std::abs(distance);

  // create a motion profiled trajectory of wheel velocities from target
  // distance
  Trajectory1D path(distance);

  // constant feedforward acceleration and feedback
  float ka = 0.02, kp = .7;

  // iterate through wheel velocities
  for (math::Pose1D pose : path.get()) {
    // fetch desired pose
    float des_dist = pose.pos;
    float des_vel = pose.vel;
    float des_acc = pose.acc;

    // feedforward velocity mapping
    float ff = 0;
    for (int n = 0; n <= 7; n++) {
      ff += args[n] * pow(des_vel, n);
    }

    // send outputs
    float out = ff + ka * des_acc +
                kp * (des_vel - (get_left_vel() + get_right_vel()) / 2);
    move(dir * out);
    pros::delay(10);
  }
}

void Driver::straight(void (*sub)(void), float distance) {
  pros::Task inside = pros::Task(sub);

  // seperate sign from magnitude of target value (for feed control)
  float dir = math::Math::sgn(distance);
  distance = std::abs(distance);

  // create a motion profiled trajectory of wheel velocities from target
  // distance
  Trajectory1D path(distance);

  // constant feedforward acceleration and feedback
  float ka = 0.02, kp = .7;

  // iterate through wheel velocities
  for (math::Pose1D pose : path.get()) {
    // fetch desired pose
    float des_dist = pose.pos;
    float des_vel = pose.vel;
    float des_acc = pose.acc;

    // feedforward velocity mapping
    float ff = 0;
    for (int n = 0; n <= 7; n++) {
      ff += args[n] * pow(des_vel, n);
    }

    // send outputs
    float out = ff + ka * des_acc +
                kp * (des_vel - (get_left_vel() + get_right_vel()) / 2);
    move(dir * out);
    pros::delay(10);
  }

  if (inside.get_state() == pros::E_TASK_STATE_RUNNING)
    inside.remove();
}

void Driver::follow_prim(Trajectory2D trajectory, int direction) {
  // tuning factors for path following
  float kv = 1.7, ka = 0.05, kp = .7;
  float w = 2.5;
  float time = 0;
  int lin_dir, ang_dir;

  // direction argument preferences for modifying paths on the fly
  if (direction == 1)
    lin_dir = 1, ang_dir = 1;
  if (direction == -1)
    lin_dir = -1, ang_dir = 1;
  if (direction == 2)
    lin_dir = 1, ang_dir = -1;
  if (direction == -2)
    lin_dir = -1, ang_dir = -1;

  /* iterating through the trajectory of robot poses (position, linear velocity,
   * heading, angular velocity, acceleration) */
  for (math::Pose2D pose : trajectory.get()) {

    // retrieve desired velocities and acceleration from current pose
    float des_lin_vel = pose.linear_vel * lin_dir;
    float des_ang_vel = pose.angular_vel * ang_dir;
    float des_acc = pose.linear_acc * lin_dir;

    /* convert linear and angular velocities to final left and right wheel
     * velocities, from the differential drive kinematics, where 'w' is the
     * responsiveness to desired change in heading and angular velocity */
    float left_control = des_lin_vel + w * des_ang_vel * trackwidth / 2;
    float right_control = des_lin_vel - w * des_ang_vel * trackwidth / 2;

    /* apply tuning factors to left and right control outputs, scaled based on a
     * gaussian distribution curve, where 'ka' is the feedforward acceleration
     * factor, and 'kp' is the feedback factor */
    float left_ff = 0, right_ff = 0;
    for (int n = 0; n <= 7; n++) {
      left_ff += args[n] * pow(left_control, n);
      right_ff += args[n] * pow(right_control, n);
    }

    float left_out =
        left_ff + ka * des_acc + kp * (left_control - get_left_vel());
    float right_out =
        right_ff + ka * des_acc + kp * (right_control - get_right_vel());

    move_left(left_out);
    move_right(right_out);
    pros::delay(10);
  }
}

void Driver::follow_prim(void (*sub)(void), Trajectory2D trajectory,
                         int direction) {

  pros::Task inside = pros::Task(sub);
  // tuning factors for path following
  float kv = 1.7, ka = 0.05, kp = .7;
  float w = 2.5;
  float time = 0;
  int lin_dir, ang_dir;

  // direction argument preferences for modifying paths on the fly
  if (direction == 1)
    lin_dir = 1, ang_dir = 1;
  if (direction == -1)
    lin_dir = -1, ang_dir = 1;
  if (direction == 2)
    lin_dir = 1, ang_dir = -1;
  if (direction == -2)
    lin_dir = -1, ang_dir = -1;

  /* iterating through the trajectory of robot poses (position, linear velocity,
   * heading, angular velocity, acceleration) */
  for (math::Pose2D pose : trajectory.get()) {

    // retrieve desired velocities and acceleration from current pose
    float des_lin_vel = pose.linear_vel * lin_dir;
    float des_ang_vel = pose.angular_vel * ang_dir;
    float des_acc = pose.linear_acc * lin_dir;

    /* convert linear and angular velocities to final left and right wheel
     * velocities, from the differential drive kinematics, where 'w' is the
     * responsiveness to desired change in heading and angular velocity */
    float left_control = des_lin_vel + w * des_ang_vel * trackwidth / 2;
    float right_control = des_lin_vel - w * des_ang_vel * trackwidth / 2;

    /* apply tuning factors to left and right control outputs, scaled based on a
     * gaussian distribution curve, where 'ka' is the feedforward acceleration
     * factor, and 'kp' is the feedback factor */
    float left_ff = 0, right_ff = 0;
    for (int n = 0; n <= 7; n++) {
      left_ff += args[n] * pow(left_control, n);
      right_ff += args[n] * pow(right_control, n);
    }

    float left_out =
        left_ff + ka * des_acc + kp * (left_control - get_left_vel());
    float right_out =
        right_ff + ka * des_acc + kp * (right_control - get_right_vel());

    move_left(left_out);
    move_right(right_out);
    pros::delay(10);
  }
  if (inside.get_state() == pros::E_TASK_STATE_RUNNING)
    inside.remove();
}

void Driver::turn_pt(math::Angle desired_heading, int rough) {
  // evalute angle to turn (initial error)
  float targ = desired_heading.radians().get();
  float curr = get_heading().get();
  math::Angle raw_ang_dist =
      math::Angle(math::Math::find_min_angle(targ, curr), math::Unit::RADIANS);

  // setup loop exit conditions and derivative conditions
  float prev = 0, tol = 0.008, tol2 = .007, timeout = 0, timeout2 = 0,
        maxtime = 2, deadband = 0.1;
  float ang, prev_ang = 0;
  if (rough == 2) tol = 0.11;
  else if (rough == 1) tol = 0.04;

  // setup usable error cases
  float err = raw_ang_dist.get();
  float err_deg = raw_ang_dist.degrees().get();
  float min = 150, max = 900, x = fabs(err_deg);

  // optimize controller efficiency by pre-checking error tolerance
  bool skip = 0;
  if (x==0) {
    x = 0.00001;
    skip = 1;
  }

  //initialize PID controller with autotuned gains
  utility::PID controller(fmin(max, fmax(min, prop_filter[0] + prop_filter[1]*x 
                          + prop_filter[2]*1/x + prop_filter[3]*pow(x, prop_filter[4]) 
                          + prop_filter[5]*pow(prop_filter[6], x)))
  ,0,1400,deadband);

  while (!skip) { // if initial error is greater than tolerance
    // reevaluate angle to turn (error)
    targ = desired_heading.radians().get();
    curr = get_heading().get();
    math::Angle raw_ang_dist = math::Angle(
        math::Math::find_min_angle(targ, curr), math::Unit::RADIANS);

    // retrieve output from constructed PID with autotuned gains
    err = raw_ang_dist.get();
    float out = controller.get_output(err, prev, 1);
    prev = err;

    // send outputs
    move_left(out);
    move_right(-out);

    // determine angular velocity (for exit conditions)
    ang = curr;
    float del_ang = ang - prev_ang;
    prev_ang = ang;

    // evaluate error tolerance and increment settle time
    if (fabs(err) < tol)
      timeout++;
    else
      timeout = 0;

    // evaluate angular velocity timeout and increment settle time
    if (fabs(del_ang) < tol2)
      timeout2++;
    else
      timeout2 = 0;

    // check for settle conditions and exit if met
    if (rough == 4) {
      if (timeout >= maxtime || timeout2 >= maxtime + 8)
        break;
    }
    else if (rough < 1) {
      if (timeout >= maxtime || timeout2 >= maxtime + 15)
        break;
    }
    else if (rough < 2) {
      if (timeout >= maxtime || timeout2 >= maxtime + 15)
        break;
    }
    else {
      if (timeout >= maxtime)
        break;
    }
    pros::delay(10);
  }
}

void Driver::turn_pt(void (*sub)(void), math::Angle desired_heading, int rough) {
  pros::Task inside = pros::Task(sub);
  // evalute angle to turn (initial error)
  float targ = desired_heading.radians().get();
  float curr = get_heading().get();
  math::Angle raw_ang_dist =
      math::Angle(math::Math::find_min_angle(targ, curr), math::Unit::RADIANS);

  // setup loop exit conditions and derivative conditions
  float prev = 0, tol = 0.008, tol2 = .007, timeout = 0, timeout2 = 0,
        maxtime = 2, deadband = 0.1;
  float ang, prev_ang = 0;
  if (rough == 2) tol = 0.11;
  else if (rough == 1) tol = 0.04;

  // setup usable error cases
  float err = raw_ang_dist.get();
  float err_deg = raw_ang_dist.degrees().get();
  float min = 150, max = 900, x = fabs(err_deg);

  // optimize controller efficiency by pre-checking error tolerance
  bool skip = 0;
  if (x==0) {
    x = 0.00001;
    skip = 1;
  }

  //initialize PID controller with autotuned gains
  utility::PID controller(fmin(max, fmax(min, prop_filter[0] + prop_filter[1]*x 
                          + prop_filter[2]*1/x + prop_filter[3]*pow(x, prop_filter[4]) 
                          + prop_filter[5]*pow(prop_filter[6], x)))
  ,0,1400,deadband);

  while (!skip) { // if initial error is greater than tolerance
    // reevaluate angle to turn (error)
    targ = desired_heading.radians().get();
    curr = get_heading().get();
    math::Angle raw_ang_dist = math::Angle(
        math::Math::find_min_angle(targ, curr), math::Unit::RADIANS);

    // retrieve output from constructed PID with autotuned gains
    err = raw_ang_dist.get();
    float out = controller.get_output(err, prev, 1);
    prev = err;

    // send outputs
    move_left(out);
    move_right(-out);

    // determine angular velocity (for exit conditions)
    ang = curr;
    float del_ang = ang - prev_ang;
    prev_ang = ang;

    // evaluate error tolerance and increment settle time
    if (fabs(err) < tol)
      timeout++;
    else
      timeout = 0;

    // evaluate angular velocity timeout and increment settle time
    if (fabs(del_ang) < tol2)
      timeout2++;
    else
      timeout2 = 0;

    // check for settle conditions and exit if met
    if (rough == 4) {
      if (timeout >= maxtime || timeout2 >= maxtime + 8)
        break;
    }
    else if (rough < 1) {
      if (timeout >= maxtime || timeout2 >= maxtime + 15)
        break;
    }
    else if (rough < 2) {
      if (timeout >= maxtime || timeout2 >= maxtime + 15)
        break;
    }
    else {
      if (timeout >= maxtime)
        break;
    }
    pros::delay(10);
  }

  if (inside.get_state() == pros::E_TASK_STATE_RUNNING)
    inside.remove();
}

void Driver::turn_rel(math::Angle desired_heading, int rough) {
  // evalute angle to turn (initial error)
  float targ = desired_heading.radians().get();
  set_relative_heading(math::Angle(0, math::Unit::DEGREES));
  float curr = get_relative_heading().get();
  math::Angle raw_ang_dist = math::Angle(targ - curr, math::Unit::RADIANS);

  // setup loop exit conditions and derivative conditions
  float prev = 0, tol = 0.008, tol2 = .007, timeout = 0, timeout2 = 0,
        maxtime = 2, deadband = 0.1;
  float ang, prev_ang = 0;
  if (rough == 2) tol = 0.11;
  else if (rough == 1) tol = 0.04;

  // setup usable error cases
  float err = raw_ang_dist.get();
  float err_deg = raw_ang_dist.degrees().get();
  float min = 150, max = 900, x = fabs(err_deg);

  // optimize controller efficiency by pre-checking error tolerance
  bool skip = 0;
  if (x==0) {
    x = 0.00001;
    skip = 1;
  }

  //initialize PID controller with autotuned gains
  utility::PID controller(fmin(max, fmax(min, prop_filter[0] + prop_filter[1]*x 
                          + prop_filter[2]*1/x + prop_filter[3]*pow(x, prop_filter[4]) 
                          + prop_filter[5]*pow(prop_filter[6], x)))
  ,0,1400,deadband);

  while (!skip) { // if initial error is greater than tolerance
    // reevaluate angle to turn (error)
    targ = desired_heading.radians().get();
    curr = get_relative_heading().get();
    math::Angle raw_ang_dist = math::Angle(targ - curr, math::Unit::RADIANS);

    // retrieve output from constructed PID with autotuned gains
    err = raw_ang_dist.get();
    float out = controller.get_output(err, prev, 1);
    prev = err;

    // send outputs
    move_left(out);
    move_right(-out);

    // determine angular velocity (for exit conditions)
    ang = curr;
    float del_ang = ang - prev_ang;
    prev_ang = ang;

    // evaluate error tolerance and increment settle time
    if (fabs(err) < tol)
      timeout++;
    else
      timeout = 0;

    // evaluate angular velocity timeout and increment settle time
    if (fabs(del_ang) < tol2)
      timeout2++;
    else
      timeout2 = 0;

    // check for settle conditions and exit if met
    if (rough < 2) {
      if (timeout >= maxtime || timeout2 >= maxtime + 15)
        break;
    }
    else {
      if (timeout >= maxtime)
        break;
    }
    pros::delay(10);
  }
}

void Driver::turn_swing(math::Angle desired_heading, int direction) {
  // evalute angle to turn (initial error)
  float targ = desired_heading.radians().get();
  float curr = get_heading().get();
  math::Angle raw_ang_dist =
      math::Angle(math::Math::find_min_angle(targ, curr), math::Unit::RADIANS);

  // setup usable error cases
  float err = raw_ang_dist.get();
  float err_deg = raw_ang_dist.degrees().get();
  float min = 140, x = fabs(err_deg);

  // optimize controller efficiency by pre-checking error tolerance
  bool skip = 0;
  if (x == 0) {
    x = 0.00001;
    skip = 1;
  }

  // initialize PID controller with autotuned gains
  utility::PID controller(
      fmax(min, prop_filter[0] + prop_filter[1] * x + prop_filter[2] * 1 / x +
                    prop_filter[3] * pow(x, prop_filter[4])),
      0,
      1400);

  // setup loop exit conditions and derivative conditions
  float prev = 0, tol = 0.06, tol2 = .007, timeout = 0, timeout2 = 0,
        maxtime = 2;
  float ang, prev_ang = 0;

  while (!skip) { // if initial error is greater than tolerance
    // reevaluate angle to turn (error)
    targ = desired_heading.radians().get();
    curr = get_heading().get();
    math::Angle raw_ang_dist = math::Angle(
        math::Math::find_min_angle(targ, curr), math::Unit::RADIANS);

    // retrieve output from constructed PID with autotuned gains
    err = raw_ang_dist.get();
    float out = controller.get_output(err, prev, 1);
    prev = err;

    // left/right swing output conditions
    if (direction > 0) { // if left specified
      // move left, hold right
      move_left(fabs(out) * math::Math::sgn(err));
      move_right(0);
      set_left_brake(utility::BrakeType::COAST);
      set_right_brake(utility::BrakeType::BRAKE);
    } else { // if right specified
      // move right, hold left
      move_left(0);
      move_right(-fabs(out) * math::Math::sgn(err));
      set_left_brake(utility::BrakeType::BRAKE);
      set_right_brake(utility::BrakeType::COAST);
    }

    // determine angular velocity (for exit conditions)
    ang = curr;
    float del_ang = ang - prev_ang;
    prev_ang = ang;

    // evaluate error tolerance and increment settle time
    if (fabs(err) < tol)
      timeout++;
    else
      timeout = 0;

    // evaluate angular velocity timeout and increment settle time
    if (fabs(del_ang) < tol2)
      timeout2++;
    else
      timeout2 = 0;

    // check for settle conditions and exit if met
    if (timeout >= maxtime || timeout2 >= maxtime + 200)
      break;

    pros::delay(10);
  }
  set_left_brake(utility::BrakeType::COAST);
  set_right_brake(utility::BrakeType::COAST);
}

void Driver::mtp(math::Vector target, int direction) {
  // initialize error-based exit conditions
  float dist_tol = 12;
  math::Vector pos_tol(.5, .5);
  float prev_ang = 0, prev_lat = 0;

  // initialize error terms and retrieve initial position
  math::Vector curr = get_relative_position();
  float dist = target.distance(curr);
  math::Angle curr_ang = get_relative_heading().radians();
  math::Vector pos_err = target - curr;
  float min_vel = .25;
  float init_ang_targ = atan2(pos_err.x, pos_err.y);
  if (direction < 0 && init_ang_targ > 0)
    init_ang_targ -= M_PI;
  else if (direction < 0 && init_ang_targ < 0)
    init_ang_targ += M_PI;
  float ang_err = math::Math::find_min_angle(init_ang_targ, curr_ang.get());
  float x = fabs(ang_err * 180 / M_PI);

  utility::PID angular(
      fmax(180, prop_filter[0] + prop_filter[1] * x + prop_filter[2] * 1 / x +
                    prop_filter[3] * pow(x, prop_filter[4])),
      0,
      derivative_filter[0] + derivative_filter[1] * x +
          derivative_filter[2] * x * x + derivative_filter[3] * x * x * x +
          derivative_filter[4] * x * x * x * x);

  // initialize movement and turning feedback controllers from static gains
  utility::PID lateral(2.75, 0, 0);

  do {
    // retrieve current pose and evaluate current state errors
    curr = get_relative_position();
    curr_ang = get_relative_heading().radians();
    dist = target.distance(curr);
    pos_err = target - curr;

    // determine angular error as differential between current heading and
    // desired heading from target position
    ang_err = math::Math::find_min_angle(init_ang_targ, curr_ang.get());

    // determine linear error as distance from target times a factor to prevent
    // drifting away from target
    float lat_err = dist * math::Math::sgn(cos(ang_err));

    // retrieve angular and linear outputs from constructed gains
    float ang_term = angular.get_output(ang_err, prev_ang, 1);
    float lat_term = lateral.get_output(lat_err, prev_lat, 1);
    prev_lat = lat_err;
    prev_ang = ang_err;

    // convert linear/angular outputs to left/right outputs through differential
    // kinematics
    float left = direction * lat_term + ang_term;
    float right = direction * lat_term - ang_term;

    /* to prevent saturation of motor velocities, preserve ratio bewtween
     * left and right sides if max velocity is exceeded */
    float ratio = fmax(fabs(left), fabs(right)) / 127;
    if (ratio > 1) {
      left /= ratio;
      right /= ratio;
    }

    // send outputs
    move_left(left);
    move_right(right);

    pros::delay(10);
  } while (!(dist < dist_tol)); // check exit conditions
}

void Driver::control() {
  // initialize output velocity and slew acceleration constants
  float v_out = 0, w_out = 0, dt = 0.01, tolerance = 0.8;
  float accel_static = 1000, vt = 0, vt_prev = 0;
  float accelw_static = 1600, wt = 0, wt_prev = 0;
  float accel, accelw;
  float max_lin = 127;
  float max_ang = 127;

  while (1) {
    // reset brake type if changed elsewhere
    set_brake(utility::BrakeType::COAST);

    // measure current time for accurate delay timing
    std::uint32_t nw = pros::millis();

    // measure input error (discrete derivative of current output)
    float ve = input::Analog::get_left_y() - v_out;
    float we = input::Analog::get_right_x() - w_out;

    if (math::Math::sgn(ve) < 0)  // during linear deceleration
      accel = accel_static + 300; // increase linear acceleration
    else
      accel = accel_static;         // revert linear acceleration
    if (math::Math::sgn(we) < 0)    // during angular deceleration
      accelw = accelw_static + 300; // increase angular acceleration
    else
      accelw = accelw_static; // revert angular acceleration

    // when linear error is over tolerance (prevents drifting)
    if (fabs(ve) > tolerance) {
      // accumulate linear velocity from specified acceleration and restrict it
      // to static max velocity
      vt += math::Math::sgn(ve) * accel * dt;
      vt = math::Math::sgn(vt) * fmin(fabs(vt), max_lin);

      // evaluate velocity differential and update output/error values
      float dv = vt - vt_prev;
      v_out += dv;
      ve -= dv;
      vt_prev = vt;
    }

    // when angular error is over tolerance (prevents drifting)
    if (fabs(we) > tolerance) {
      // accumulate angular velocity from specified acceleration and restrict it
      // to static max velocity
      wt += math::Math::sgn(we) * accelw * dt;
      wt = math::Math::sgn(wt) * fmin(fabs(wt), max_ang);

      // evaluate velocity differential and update output/error values
      float dw = wt - wt_prev;
      w_out += dw;
      we -= dw;
      wt_prev = wt;
    }

    // prioritize turning through 1:1 turn stick mapping
    float n = input::Analog::get_right_x();

    // convert linear/angular velocities to left/right velocites through
    // differential kinematics
    float lvel = v_out + n;
    float rvel = v_out - n;

    /* to prevent saturation of motor velocities, preserve ratio bewtween
     * left and right sides if max velocity is exceeded */
    float ratio = std::max(std::abs(lvel), std::abs(rvel)) / 127;
    if (ratio > 1) {
      lvel /= ratio;
      rvel /= ratio;
    }

    float v_ = input::Analog::get_left_y();
    float w_ = input::Analog::get_right_x();
    float k = 1.05;

    float v_map = map(v_), w_map = map(w_);
    float left = v_map + k*w_map;
    float right = v_map - k*w_map;

    float ratio2 = std::max(std::abs(left), std::abs(right)) / 127;
    if (ratio2 > 1) {
      left /= ratio2;
      right /= ratio2;
    }

    // send outputs
    move_left(left);
    move_right(right);

    pros::Task::delay_until(&nw, 1000 * dt);
  }
}

void Driver::set_accel_time(float a) { accel_time = a; }

void Driver::set_driver_velocity(float v) { driver_max_velocity = v; }

void Driver::set_controller_tuning(std::string _type, float _curve,
                                   std::string _map, float _accel_time,
                                   float _brake_thresh, float _turn_sens) {
  control_type = _type;
  curve = _curve;
  map_type = _map;
  accel_time = _accel_time;
  brake_thresh = _brake_thresh;
  turn_sens = _turn_sens;
}

float Driver::map(float x) {
  if (x == 0)
    return 0;
  return (127 * pow(std::abs(x), curve)) / pow(127, curve) * (x / std::abs(x));
}

float Driver::logistic_map(float x) {
  if (x < 0)
    return -127 / (1 + exp(-curve * (-x - 63.5)));
  if (x > 0)
    return 127 / (1 + exp(-curve * (x - 63.5)));
  return 0;
}

void none() {}

} // namespace lib::control
