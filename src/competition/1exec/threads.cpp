#include "main.h"

namespace sys_task {

void run_systems() {
  while (1) {
    std::uint32_t nw = pros::millis();

    cata.set_brake_mode(BrakeType::HOLD);
    if (cata_req)
      cata.move(127 * .8);
    else
      cata.brake();

    intake.set_brake_mode(BrakeType::HOLD);
    if (intake_req)
      intake.move(intake_speed);
    else if (intake_rev_req)
      intake.move(-intake_speed);
    else
      intake.brake();

    front_wings.set(front_wings_req);
    hang1.set(hang_req);
    hang2.set(hang_req);
    left_wing.set(left_wing_req || wings_req);
    right_wing.set(right_wing_req || wings_req);

    pros::Task::delay_until(&nw, 5);
  }
}

void run_input() {
  while (1) {
    std::uint32_t nw = pros::millis();
    cata_req = Button::a_pressed;
    intake_req = Digital::pressing(Button::R1);
    intake_rev_req = Digital::pressing(Button::L1);
    front_wings_req = Digital::pressing(Button::R2);
    wings_req = Digital::pressing(Button::L2);
    hang_req = Button::b_pressed;
    pros::Task::delay_until(&nw, 5);
  }
}

bool cata_req = 0, front_wings_req = 0, wings_req = 0, hang_req = 0,
     left_wing_req = 0, right_wing_req = 0, intake_req = 0, intake_rev_req = 0,
     accel_off = 0;
float intake_speed = 127;

} // namespace sys_task