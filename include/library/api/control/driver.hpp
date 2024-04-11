/**
 * \file driver.hpp
 *
 * Driver class, controls all chassis movements, both autonomous and controlled.
 *
 * @author 3135B
 */

#pragma once
#include "../../util.hpp"
#include "../input/analog.hpp"
#include "../input/button.hpp"
#include "../input/digital.hpp"
#include "../math/angle.hpp"
#include "../math/math.hpp"
#include "../math/vector.hpp"
#include "../util/braketype.hpp"
#include "../util/pid.hpp"
#include "../util/positiontracker.hpp"
#include "../util/timer.hpp"
#include "angulartrajectory.hpp"
#include "chassis.hpp"
#include "trajectory1d.hpp"
#include "trajectory2d.hpp"

namespace lib::control {

class Driver : public utility::PositionTracker {
public:
  /**
   * Constructs a Driver as a Chassis, with left and right ports, and as an
   * Odometry instance, with sensor ports.
   * @param left_ports : set of left ports
   * @param right_ports : set of right ports
   * @param wheel : diameter of drive wheels
   * @param speed : speed of chassis in RPM
   * @param imu1 : imu sensor port
   * @param trackw : robot track width
   */
  Driver(std::vector<int> left_ports, std::vector<int> right_ports, float wheel,
         float speed, int imu1, float trackw);

  void follow_prim(Trajectory2D, int direction);
  void follow_prim(void (*sub)(void), Trajectory2D, int direction);
  void follow_feed(Trajectory2D trajectory, int direction);
  void straight(float distance);
  void straight(void (*sub)(void), float distance);
  void turn_pt(math::Angle desired_heading);
  void turn_swing(math::Angle desired_heading, int direction);

  /**
   * Commands the robot using the controller for the driver control period.
   */
  void control(void);

  /**
   * Changes the acceleration time of the Driver.
   * @param a : new acceleration time
   */
  void set_accel_time(float a);

  /**
   * Changes the maximum velocity of the Driver.
   * @param v : new maximum velocity
   */
  void set_driver_velocity(float v);

  /**
   * Sets the controller preferences for driving.
   * @param _type : type of control (Arcade or Tank)
   * @param _curve : curve constant for acceleration (0 < k <= 10)
   * @param _accel_time : 1 / time given to accelerate
   * @param _brake_tresh : minimum motor voltage required for power cut off
   * @param _turn_sens : factor that affects the sensitivity of turning (only
   * applicable for arcade, 0 < _turn_sens < 1)
   */
  void set_controller_tuning(std::string _type, float _curve, std::string _map,
                             float _accel_time, float _brake_thresh,
                             float _turn_sens);

  float left_final_velocity;
  float right_final_velocity;

private:
  /**
   * Maps a linear output value to an exponential curve value, based on the
   * curve instance variable.
   */
  float map(float x);

  /**
   * Maps a linear output value to a logistic curve value, based on the
   * curve instance variable.
   */
  float logistic_map(float x);

  /**
   * Maps a linear output value to a gaussian curve value, based on the
   * curve instance variable.
   */
  float gaussian_map(float x);

  std::string control_type;
  float curve;
  std::string map_type;
  float accel_time;
  float brake_thresh;
  float trackwidth;
  float turn_sens;
  float left_y_error;
  float left_y_output;
  float left_x_error;
  float left_x_output;
  float right_y_error;
  float right_y_output;
  float right_x_error;
  float right_x_output;
  float left_velocity;
  float right_velocity;
  float driver_max_velocity;
  static inline float args[] = {3.96476,       6.8259,         -0.945961,
                                0.076107,      -0.00311266,    0.0000674742,
                                -0.0000007372, 0.0000000031934};
  static inline float prop_filter[] = {232.389, -0.166599, 3463.24, -757.926,
                                       -0.438566};
  static inline float derivative_filter[] = {987.966, 1.35596, -0.00542918,
                                             0.00000594226, 0.000000037485};
};

/**
 * Default async function.
 */
void none(void);

} // namespace lib::control