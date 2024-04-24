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
      auton_str = "none            ";
      break;
    case 1:
      auton_str = "safe 6b         ";
      break;
    case 2:
      auton_str = "safe 6b touch   ";
      break;
    case 3:
      auton_str = "safe 4b touch   ";
      break;
    case 4:
      auton_str = "rush 6b mid     ";
      break;
    case 5:
      auton_str = "rush 6b far     ";
      break;
    case 6:
      auton_str = "rush 6b flick   ";
      break;
    case 7:
      auton_str = "safe close      ";
      break;
    case 8:
      auton_str = "safe close alt  ";
      break;
    case 9:
      auton_str = "rush close wp   ";
      break;
    case 10:
      auton_str = "rush close      ";
      break;
    case 11:
      auton_str = "rush close far  ";
      break;
    case 12:
      auton_str = "rush close push ";
      break;
    case 13:
      auton_str = "skills          ";
      break;
    }
    master.print(0, 0, "%s", auton_str);
    pros::lcd::print(0, "%s", auton_str);

    pros::Task::delay_until(&nw, 15);
  }
}

void run_auton() {
  switch (selected_auton) {
  case 0:
    sys_task::intake_req = 1;
    pros::delay(1000);
    sys_task::intake_req = 0;
    break;
  case 1:
    auton::safe_6b();
    break;
  case 2:
    auton::safe_6b_touch();
    break;
  case 3:
    auton::safe_4b_touch();
    break;
  case 4:
    auton::rush_6b_mid();
    break;
  case 5:
    auton::rush_6b_far();
    break;
  case 6:
    auton::rush_6b_flick();
    break;
  case 7:
    auton::safe_close();
    break;
  case 8:
    auton::safe_close_alt();
    break;
  case 9:
    auton::rush_close_wp();
    break;
  case 10:
    auton::rush_close();
    break;
  case 11:
    auton::rush_close_far();
    break;
  case 12:
    auton::rush_close_push();
    break;
  case 13:
    auton::skills();
    break;
  }
}

uint32_t max_time = 14998;
unsigned int selected_auton = 0, num_autons = 13;

}; // namespace auton