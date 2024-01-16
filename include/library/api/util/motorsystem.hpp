/**
 * \file motorsystem.hpp
 *
 * A MotorSystem is constructed based on a Chassis and shares the same
 * characteristics.
 *
 * @author 3135B
 */

#pragma once
#include "../../util.hpp"
#include "../control/chassis.hpp"

namespace lib::utility {

class MotorSystem : public lib::control::Chassis {
public:
  /**
   * Constructs a MotorSystem as a Chassis.
   * @param left_ports : left motors in system
   * @param right_motors : right motors in system
   */
  MotorSystem(std::vector<int> left_ports, std::vector<int> right_ports);

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

  /**
   * Retrieves the Rotation sensor value from the mechanism.
   * @return rotation sensor value
   */
  float get(void);

  /**
   * Resets the Rotation sensor value to 0.
   */
  void reset(void);
};

}; // namespace lib::utility