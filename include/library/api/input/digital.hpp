/**
 * \file digital.hpp
 *
 * Digital class, contains static methods to access the status of all
 * digital buttons.
 *
 * @author 3135B
 */

#pragma once
#include "../../util.hpp"
#include "button.hpp"
#include "globals.hpp"

namespace lib::input {

class Digital {
public:
  /**
   * Retrieves if a given button is being pressed.
   * @param b : button being checked
   * @return if the button is being pressed
   */
  static bool pressing(char b);

  /**
   * Retrieves if a given button has been pressed.
   * @param b : button being checked
   * @return if the button has been pressed
   */
  static bool pressed(char b);

  /**
   * Method used in a thread to update button statuses when they are
   * pressed during driver control.
   */
  static void toggle_thread(void *param);
};

}; // namespace lib::input