#include "library/api/util/digitalsystem.hpp"

namespace lib::utility {

DigitalSystem::DigitalSystem(char port) {
  mech = new pros::ADIDigitalOut(port);
}

void DigitalSystem::set(bool val) { mech->set_value(val); }

}; // namespace lib::utility