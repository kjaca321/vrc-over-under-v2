/**
 * \file timer.hpp
 *
 * Timer class, used for timing various tasks and events throughout a match.
 *
 * @author 3135B
 */

#pragma once
#include "../../util.hpp"

namespace lib::utility {

class Timer {
public:
  /**
   * Retrieves the time passed since the program started in milliseconds.
   */
  static uint32_t get(void);

  /**
   * Retrieves the time when called and stores it in the desired key.
   */
  static void place_marker(std::string key);

  /**
   * Retrieves the change in time from the desired key.
   */
  static uint32_t get_from_marker(std::string key);

  /**
   * Retrieves the time when called and stores it in the desired final key.
   */
  static void place_final_marker(std::string key);

  /**
   * Removes all elements from the final_markers map.
   */
  static void clear_final_markers(void);

  /**
   * Removes an elements from the final_markers map, from a key.
   */
  static void remove_final_marker(std::string key);

private:
  /**
   * Map of time markers, based on std::string keys.
   */
  static inline std::map<std::string, uint32_t> markers;

  /**
   * Map of final time markers, based on std::string keys.
   */
  static inline std::map<std::string, uint32_t> final_markers;
};

}; // namespace lib::utility