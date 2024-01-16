/**
 * \file timer.cpp
 *
 * Timer class, used for timing various tasks and events throughout a match.
 *
 * @author 3135B
 */

#include "library/api/util/timer.hpp"

namespace lib::utility {

uint32_t Timer::get() { return pros::millis(); }

void Timer::place_marker(std::string key) { markers[key] = pros::millis(); }

void Timer::place_final_marker(std::string key) {
  if (final_markers.find(key) == final_markers.end())
    markers[key] = pros::millis();
}

uint32_t Timer::get_from_marker(std::string key) {
  if (final_markers.find(key) != final_markers.end())
    return pros::millis() - final_markers[key];
  if (markers.find(key) != markers.end())
    return pros::millis() - markers[key];
  throw std::runtime_error("Key not found!");
}

void Timer::clear_final_markers() { final_markers.clear(); }

void Timer::remove_final_marker(std::string key) { final_markers.erase(key); }

}; // namespace lib::utility