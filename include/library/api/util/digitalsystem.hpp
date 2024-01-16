/**
 * \file digitalsystem.hpp
 *
 * A DigitalSystem is constructed from a 3-wire port character and controls
 * the action of a ADIDigitalOut port.
 *
 * @author 3135B
 */

#pragma once
#include "../../util.hpp"

namespace lib::utility {

class DigitalSystem {
public:
  /**
   * Constructs a DigitalSystem with a 3-wire port.
   * @param port : 3-wire port location
   */
  DigitalSystem(char port);

  /**
   * Changes the state of the DigitalSystem.
   * @param val : new state
   */
  void set(bool val);

private:
  pros::ADIDigitalOut *mech;
};

}; // namespace lib::utility