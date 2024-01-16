/**
 * \file vector.hpp
 *
 * A Vector has an x and a y. The class contains several immutable methods to
 * retrieve aspects of the Vector object.
 *
 * @author 3135B
 */

#pragma once
#include "../../util.hpp"

namespace lib::math {

class Vector {
public:
  /**
   * Constructs a default Vector with an x and y of 0.
   */
  Vector(void);

  /**
   * Constructs a Vector with a given x and y.
   * @param inx : x of Vector
   * @param iny : y of Vector
   */
  Vector(float inx, float iny);

  /**
   * Overloads the (+) operator to add two Vectors.
   * @param rhs : Vector being added
   * @return new Vector that is the sum
   */
  Vector operator+(const Vector &rhs);

  /**
   * Overloads the (-) operator to subtract another Vector from the Vector.
   * @param rhs : Vector being subtracted
   * @return new Vector that is the difference
   */
  Vector operator-(const Vector &rhs);

  /**
   * Overloads the (*) operator to perform the dot product on two Vectors.
   * @param rhs : other Vector
   * @return the dot product
   */
  float operator*(const Vector &rhs);

  /**
   * Overloads the (*) operator to perform scalar multiplication on the Vector.
   * @param rhs : scalar being multiplied
   * @return the multiplied Vector
   */
  Vector operator*(const float num);

  /**
   * Retrieves the magnitude of the Vector.
   * @return the magnitude of the Vector
   */
  float magnitude(void);

  /**
   * Retrieves the unit Vector corresponding to the Vector.
   * @return a normalized Vector corresponding to the original
   */
  Vector normalize(void);

  /**
   * Retrieves a std::string representation of the current state of the Vector
   * object.
   * @return the current state of the Vector
   */
  std::string to_string(void);

  /**
   * Retrieves if the Vector's fields equal another's fields.
   * @param rhs : Other Vector to compare
   * @return if two Vectors are equivalent
   */
  bool equals(Vector rhs);

  /**
   * Retrieves the distance between two Vectors.
   * @param rhs : other Vector
   * @return distance between the Vector and another
   */
  float distance(Vector rhs);

  /**
   * Retrieves if the Vector is in an std::vector<Vector>.
   * @param vec : std::vector<Vector> to search in
   * @return if the Vector is in the std::vector<Vector>
   */
  bool is_in(std::vector<Vector> vec);

  /**
   * Retrieves the closest (in terms of distance) Vector in an
   * std::vector<Vector> to the Vector.
   * @param vec : std::vector<Vector> to search in
   * @return closest Vector in the std::vector<Vector>
   */
  Vector closest_in(std::vector<Vector> vec);

  /**
   * Retrieves the index of the Vector in an std::vector<Vector>, or -1 if not
   * found.
   * @param vec : std::vector<Vector> to search in
   * @return index of the Vector in the std::vector<Vector>
   */
  int find_idx_in(std::vector<Vector> vec);

  float x, y;
};

}; // namespace lib::math