/**
 * \file positiontracker.hpp
 *
 * PositionTracker class, contains static methods to track the position of the
 * robot.
 *
 * @author 3135B
 */

#pragma once
#include "../control/chassis.hpp"
#include "../math/angle.hpp"
#include "../math/unit.hpp"
#include "../math/vector.hpp"

namespace lib::utility {

class PositionTracker : public control::Chassis {
public:
  /**
   * Constructs an PositionTracker instance based on a Chassis and one
   * inertial sensor, as well as wheel size and rpm.
   * @param left_ports : set of left ports
   * @param right_ports : set of right ports
   * @param wheel : wheel size
   * @param speed : robot rpm
   * @param imu1 : imu sensor port
   */
  PositionTracker(std::vector<int> left_ports, std::vector<int> right_ports,
                  float wheel, float speed, int imu1);

  /**
   * Method that runs the position tracking algorithm for the
   * autonomous period.
   */
  void run_tracker(void);

  /**
   * Alters the starting position of the robot based on arguments.
   * @param pos : new position of robot
   * @param heading : new angle of robot
   */
  void set_position(math::Vector pos, math::Angle heading);

  /**
   * Retrieves the current position of the robot.
   * @return the current robot position
   */
  math::Vector get_position(void);

  /**
   * Changes the heading of the remapped IMU using the reset(...) method.
   * @param a : new angle of IMU
   */
  void set_heading_remap(math::Angle a);

  /**
   * Retrieves the Angle of the remapped IMU sensor in radians.
   * @return Angle of the IMU
   */
  math::Angle get_heading_remap(void);

  /**
   * Changes the heading of the IMU using the reset(...) method.
   * @param a : new angle of IMU
   */
  void set_heading(math::Angle a);
  void set_relative_heading(math::Angle a);

  /**
   * Retrieves the Angle of the IMU sensor in radians.
   * @return Angle of the IMU
   */
  math::Angle get_heading(void);
  math::Angle get_relative_heading(void);

  /**
   * Retrieves the relative linear position of the robot.
   * @return relative distance
   */
  float get_relative_displacement(void);

  /**
   * Sets the relative linear position of the robot.
   * @param distance : new relative distance
   */
  void set_relative_displacement(float distance);

  /**
   * Retrieves the relative 2D position of the robot.
   * @return relative position
   */
  math::Vector get_relative_position(void);

  /**
   * Sets the relative 2D position of the robot.
   * @param pos : new relative distance
   */
  void set_relative_position(math::Vector pos);

  float get_left_vel(void);
  float get_right_vel(void);
  float get_left_acc(void);
  float get_right_acc(void);
  float get_angular_vel(void);
  float get_angular_acc(void);

  /**
   * Calibrates the Odometry sensors.
   */
  void setup(void);

private:
  /**
   * Retrieves the averaged raw position of the left side of the drive.
   * @return raw left position
   */
  float get_raw_left_pos(void);

  /**
   * Retrieves the averaged raw position of the right side of the drive.
   * @return raw right position
   */
  float get_raw_right_pos(void);

  /**
   * Resets the raw position of the left side of the drive.
   */
  void reset_left_side(void);

  /**
   * Resets the raw position of the left side of the drive.
   */
  void reset_right_side(void);

  float previous;
  float prev_heading;
  float prev_left;
  float prev_right;
  float prev_left_vel;
  float prev_right_vel;
  float relative_distance;
  math::Angle heading;
  math::Angle relative_heading;
  math::Angle prev_heading_i;
  math::Vector position;
  math::Vector relative_position;
  pros::Imu *imu;
  pros::Imu *imu_remap;
  float wheel_size;
  float odom_left_vel;
  float odom_right_vel;
  float odom_left_acc;
  float odom_right_acc;
  float x_vel, y_vel;
  float rpm;
  float angular_vel;
  float prev_angular_vel;
  float angular_acc;
  float odom_dt;
};

}; // namespace lib::utility