/**
 * \file pid.hpp
 *
 * A PID controller is constructed with three gains and a bias.
 *
 * @author 3135B
 */

#pragma once
#include "../../util.hpp"

namespace lib::utility {

class PID {
public:
  /**
   * Constructs a PID with all gains set to 0.
   */
  PID(void);

  /**
   * Constructs a PID with paramterized kP, kI, and kD gains, and a default
   * bias value.
   * @param p : kP constant of PID
   * @param i : kI constant of PID
   * @param d : kD constant of PID
   */
  PID(float p, float i, float d);

  /**
   * Constructs a PID with paramterized kP, kI, and kD gains, and a custom bias
   * value.
   * @param p : kP constant of PID
   * @param i : kI constant of PID
   * @param d : kD constant of PID
   * @param b : custom bias of PID
   */
  PID(float p, float i, float d, float b);

  /**
   * Retrieves the control output for the PID loop, based on the error.
   * @param error : current error of system
   * @param pre_error : previous iteration error of system
   * @param dt : change in time
   * @return output of PID
   */
  float get_output(float error, float pre_error, uint32_t dt);

  /**
   * Updates the k values of the PID loop.
   * @param p : new kP constant of PID
   * @param i : new kI constant of PID
   * @param d : new kD constant of PID
   */
  void set_gains(float p, float i, float d);

  /**
   * Updates the k values and bias of the PID loop.
   * @param p : new kP constant of PID
   * @param i : new kI constant of PID
   * @param d : new kD constant of PID
   * @param b : new custom bias of PID
   */
  void set_gains(float p, float i, float d, float b);

  /**
   * Sets all gains to 0.
   */
  void reset(void);

  /**
   * Resets the integral of the loop.
   */
  void reset_integral(void);

  float kP;
  float kI;
  float kD;
  float bias;
  float integral;
};

}; // namespace lib::utility