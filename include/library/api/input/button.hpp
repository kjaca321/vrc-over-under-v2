/**
 * \file button.hpp
 *
 * Button class, contains variables that represent all the digital buttons
 * on the controller and their status.
 *
 * @author 3135B
 */

#pragma once

namespace lib::input {

class Button {
public:
  /**
   * Digital button 'A', and its pressed status.
   */
  static inline const char A = 'a';
  static bool a_pressed;

  /**
   * Digital button 'B', and its pressed status.
   */
  static inline const char B = 'b';
  static bool b_pressed;

  /**
   * Digital button 'X', and its pressed status.
   */
  static inline const char X = 'x';
  static bool x_pressed;

  /**
   * Digital button 'Y', and its pressed status.
   */
  static inline const char Y = 'y';
  static bool y_pressed;

  /**
   * Digital button 'UP', and its pressed status.
   */
  static inline const char UP = 'u';
  static bool up_pressed;

  /**
   * Digital button 'DOWN', and its pressed status.
   */
  static inline const char DOWN = 'd';
  static bool down_pressed;

  /**
   * Digital button 'LEFT', and its pressed status.
   */
  static inline const char LEFT = 'l';
  static bool left_pressed;

  /**
   * Digital button 'RIGHT', and its pressed status.
   */
  static inline const char RIGHT = 'r';
  static bool right_pressed;

  /**
   * Digital button 'L1', and its pressed status.
   */
  static inline const char L1 = '1';
  static bool l1_pressed;

  /**
   * Digital button 'L2', and its pressed status.
   */
  static inline const char L2 = '2';
  static bool l2_pressed;

  /**
   * Digital button 'R1', and its pressed status.
   */
  static inline const char R1 = '3';
  static bool r1_pressed;

  /**
   * Digital button 'R2', and its pressed status.
   */
  static inline const char R2 = '4';
  static bool r2_pressed;
};

}; // namespace lib::input