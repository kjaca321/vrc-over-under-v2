/**
 * \file angle.hpp
 *
 * An Angle has a measurement and a Unit. The Angle class contains several
 * immutable methods to retrieve aspects of the Angle object.
 *
 * @author 3135B
 */

#pragma once
#include "../../util.hpp"
#include "unit.hpp"

namespace lib::math {

class Angle {
public:
  /**
   * Constructs a default Angle with a measurement of 0 and a Unit of radians.
   */
  Angle(void);

  /**
   * Constructs an Angle with a given measurement and a Unit of radians.
   * @param num : measurement of Angle
   */
  Angle(float num);

  /**
   * Constructs an Angle with a given measurement and Unit.
   * @param num : measurement of Angle
   * @param u : Unit of Angle
   */
  Angle(float num, std::string u);

  /**
   * Converts the Angle to radians and returns the new Angle in radians.
   * @return the Angle converted to radians
   */
  Angle radians(void);

  /**
   * Converts the Angle to degrees and returns the new Angle in degrees.
   * @return the Angle converted to degrees
   */
  Angle degrees(void);

  /**
   * Checks if the Angle's fields are equal to another's fields.
   * @return if the Angle is equivalent to another
   */
  bool equals(Angle);

  /**
   * Retrieves an std::string representation of the current state of the Angle.
   * @return the current state of the Angle
   */
  std::string to_string(void);

  /**
   * Retrieves the raw float angle of the Angle object, without the unit.
   * @return the raw angle
   */
  float get(void);

  /**
   * Changes the raw float angle of the Angle object, without the unit.
   * @param  a : the new raw angle
   */
  void set(float a);

  /**
   * Overloads the (+) operator to add two Angles in the unit of the primary
   * Angle.
   * @param rhs : Angle being added
   * @return new Angle that is the sum
   */
  Angle operator+(Angle &rhs);

  /**
   * Overloads the (+) operator to add a float to the Angle in the unit of the
   * primary Angle.
   * @param rhs : float being added
   * @return new Angle that is the sum
   */
  Angle operator+(const float num);

  /**
   * Overloads the (-) operator to subtract two Angles in the unit of the
   * primary Angle.
   * @param rhs : Angle being subtracted
   * @return new Angle that is the difference
   */
  Angle operator-(Angle &rhs);

  /**
   * Overloads the (-) operator to subtract a float from the Angle in the unit
   * of the primary Angle.
   * @param rhs : float being subtracted
   * @return new Angle that is the difference
   */
  Angle operator-(const float num);

  /**
   * Overloads the (*) operator to multiply two Angles in the unit of the
   * primary Angle.
   * @param rhs : Angle being multiplied
   * @return new Angle that is the product
   */
  Angle operator*(Angle &rhs);

  /**
   * Overloads the (*) operator to multiply a float by the Angle in the unit
   * of the primary Angle.
   * @param rhs : float being multiplied
   * @return new Angle that is the product
   */
  Angle operator*(const float num);

  /**
   * Overloads the (/) operator to divide two Angles in the unit of the
   * primary Angle.
   * @param rhs : Angle that is the divisor
   * @return new Angle that is the quotient
   */
  Angle operator/(Angle &rhs);

  /**
   * Overloads the (/) operator to divide a float by the Angle in the unit
   * of the primary Angle.
   * @param rhs : float that is the divisor
   * @return new Angle that is the quotient
   */
  Angle operator/(const float num);

  std::string unit;
  float angle;
};

}; // namespace lib::math