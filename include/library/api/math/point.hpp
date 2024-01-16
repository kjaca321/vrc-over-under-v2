/**
 * \file point.hpp
 *
 * A Point inherits an x and y from a Vector, as well as all immutable methods
 * from the Vector class. In addition, a Point has a linear velocity, desired
 * heading, and curvature.
 *
 * @author 3135B
 */

#pragma once
#include "../../util.hpp"
#include "angle.hpp"
#include "vector.hpp"

namespace lib::math {

class Point : public Vector {
public:
  /**
   * Constructs a default Point with an x and y of 0, as well as a linear and
   * angular velocity of 0.
   */
  Point(void);

  /**
   * Constructs a Point with a given x and y, and a linear and angular velocity
   * of 0.
   * @param inx : x of Point
   * @param iny : y of Point
   */
  Point(float inx, float iny);

  /**
   * Constructs a Point with a given x and y, and a given linear and angular
   * velocity, as well as a given curvature.
   * @param inx : x of Point
   * @param iny : y of Point
   * @param in_linear : linear velocity of Point
   * @param in_curv : curvature of Point
   */
  Point(float inx, float iny, math::Angle in_heading, float in_linear,
        float in_curv);

  /**
   * Retrieves the closest (in terms of distance) Point in an
   * std::vector<Point> to the Point.
   * @param vec : std::vector<Point> to search in
   * @return closest Point in the std::vector<Point>
   */
  Point closest_in(std::vector<Point> vec);

  /**
   * Retrieves a std::string representation of the current state of the Point
   * object.
   * @return the current state of the Point
   */
  std::string to_string_full(void);

  /**
   * Retrieves the index of the Point in an std::vector<Point>, or -1 if not
   * found.
   * @param vec : std::vector<Point> to search in
   * @return index of the Point in the std::vector<Point>
   */
  int find_idx_in_pt(std::vector<Point> vec);

  /**
   * Retrieves if the Point's fields equal another's fields.
   * @param rhs : Other Point to compare
   * @return if two Point are equivalent
   */
  bool equals_pt(Point rhs);

  float linear_velocity, curvature;
  math::Angle heading;
};

}; // namespace lib::math