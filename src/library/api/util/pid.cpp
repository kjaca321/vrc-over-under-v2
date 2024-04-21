/**
 * \file pid.cpp
 *
 * A PID controller is constructed with three gains, and can be run as a thread
 * during autonomous to control various error terms.
 *
 * @author 3135B
 */

#include "library/api/util/pid.hpp"

namespace lib::utility {

PID::PID() : kP(0), kI(0), kD(0), bias(0), integral(0) {}

PID::PID(float p, float i, float d)
    : kP(p), kI(i), kD(d), bias(0), integral(0) {}

PID::PID(float p, float i, float d, float b)
    : kP(p), kI(i), kD(d), bias(b), integral(0) {}

float PID::get_output(float error, float pre_error, uint32_t dt) {
  if (error < bias) integral += error * dt;
  else integral = 0;
  float derivative = (error - pre_error) / (float)dt;
  return kP * error + kI * integral + kD * derivative;
}

void PID::set_gains(float p, float i, float d) {
  kP = p;
  kI = i;
  kD = d;
  integral = 0;
}

void PID::set_gains(float p, float i, float d, float b) {
  kP = p;
  kI = i;
  kD = d;
  bias = b;
  integral = 0;
}

void PID::reset() {
  kP = 0;
  kI = 0;
  kD = 0;
  bias = 0;
  integral = 0;
}

void PID::reset_integral() { integral = 0; }

}; // namespace lib::utility