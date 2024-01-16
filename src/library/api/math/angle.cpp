/**
 * \file angle.cpp
 *
 * An Angle has a measurement and a Unit. The Angle class contains several
 * immutable methods to retrieve aspects of the Angle object.
 *
 * @author 3135B
 */
#include "library/api/math/angle.hpp"

namespace lib::math {

Angle::Angle(){};

Angle::Angle(float ang) : angle(ang), unit(Unit::DEGREES){};

Angle::Angle(float ang, std::string in) : angle(ang), unit(in){};

Angle Angle::radians() {
  if (this->unit == Unit::RADIANS) {
    return *this;
  } else if (this->unit == Unit::DEGREES) {
    Angle res(this->angle * M_PI / 180, Unit::RADIANS);
    return res;
  }
  throw std::runtime_error("Incorrect Unit!");
}

Angle Angle::degrees() {
  if (this->unit == Unit::DEGREES) {
    return *this;
  } else if (this->unit == Unit::RADIANS) {
    Angle res(this->angle * 180 / M_PI, Unit::DEGREES);
    return res;
  }
  throw std::runtime_error("Incorrect Unit!");
}

bool Angle::equals(Angle rhs) {
  return this->angle == rhs.angle && this->unit == rhs.unit;
}

std::string Angle::to_string() {
  return std::to_string(this->angle) + " " + this->unit;
}

float Angle::get() { return angle; }

void Angle::set(float a) { angle = a; }

Angle Angle::operator+(Angle &rhs) {
  if (this->unit == Unit::RADIANS)
    Angle conv = rhs.radians();
  else if (this->unit == Unit::DEGREES)
    Angle conv = rhs.degrees();
  else
    throw std::runtime_error("Incorrect Unit!");
  Angle res(this->angle + rhs.angle, this->unit);
  return res;
}

Angle Angle::operator+(const float rhs) {
  Angle res(this->angle + rhs, this->unit);
  return res;
}

Angle Angle::operator-(Angle &rhs) {
  if (this->unit == Unit::RADIANS)
    Angle conv = rhs.radians();
  else if (this->unit == Unit::DEGREES)
    Angle conv = rhs.degrees();
  else
    throw std::runtime_error("Incorrect Unit!");
  Angle res(this->angle - rhs.angle, this->unit);
  return res;
}

Angle Angle::operator-(const float rhs) {
  Angle res(this->angle - rhs, this->unit);
  return res;
}

Angle Angle::operator*(Angle &rhs) {
  if (this->unit == Unit::RADIANS)
    Angle conv = rhs.radians();
  else if (this->unit == Unit::DEGREES)
    Angle conv = rhs.degrees();
  else
    throw std::runtime_error("Incorrect Unit!");
  Angle res(this->angle * rhs.angle, this->unit);
  return res;
}

Angle Angle::operator*(const float rhs) {
  Angle res(this->angle * rhs, this->unit);
  return res;
}

Angle Angle::operator/(Angle &rhs) {
  if (this->unit == Unit::RADIANS)
    Angle conv = rhs.radians();
  else if (this->unit == Unit::DEGREES)
    Angle conv = rhs.degrees();
  else
    throw std::runtime_error("Incorrect Unit!");
  Angle res(this->angle / rhs.angle, this->unit);
  return res;
}

Angle Angle::operator/(const float rhs) {
  Angle res(this->angle / rhs, this->unit);
  return res;
}

}; // namespace lib::math