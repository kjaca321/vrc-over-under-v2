#include "main.h"

namespace auton {

void run_selection() {
  while (1) {
    std::uint32_t nw = pros::millis();

    if (auton_select.get_new_press()) {
      if (selected_auton == num_autons)
        selected_auton = 0;
      else
        selected_auton++;
    }

    if (auton_calibrate.get_new_press()) {
      robot.setup();
      break;
    }

    std::string auton_str;
    switch (selected_auton) {
    case 0:
      auton_str = "safe 6b     ";
      break;
    case 1:
      auton_str = "safe 6b     ";
      break;
    // case 2:
    //   auton_str = "close elims ";
    //   break;
    // case 3:
    //   auton_str = "far quals   ";
    //   break;
    // case 4:
    //   auton_str = "far elims   ";
    //   break;
    // case 5:
    //   auton_str = "skills      ";
    //   break;
    }
    master.print(0, 0, "%s", auton_str);
    pros::lcd::print(0, "%s", auton_str);

    pros::Task::delay_until(&nw, 15);
  }
}

void run_auton() {
  switch (selected_auton) {
  case 0:
    auton::safe_6b();
    break;
  case 1:
    auton::safe_6b();
    break;
  }
}

uint32_t max_time = 14998;
unsigned int selected_auton = 0, num_autons = 1;

}; // namespace auton