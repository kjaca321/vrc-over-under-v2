#include "main.h"

namespace sys_task {

void run_systems() {
  while (1) {
    std::uint32_t nw = pros::millis();
    float pct = .85;
    cata.set_brake_mode(BrakeType::COAST);
    cata2.set_brake_mode(BrakeType::COAST);
    if (cata_req || master_cata_req) {
      cata.move(127 * pct);
      cata2.move(127 * pct);
    }
    else {
      cata.move(0);
      cata2.move(0);
    }

    intake.set_brake_mode(BrakeType::HOLD);
    intake2.set_brake_mode(BrakeType::HOLD);
    if (intake_req) {
      intake.move(intake_speed);
      intake2.move(intake_speed);
    }
    else if (intake_rev_req) {
      intake.move(-intake_speed);
      intake2.move(-intake_speed);
    }
    else {
      intake.brake();
      intake2.brake();
    }

    if (Button::down_pressed) {
      robot.set_brake(BrakeType::HOLD);
    }
    else robot.set_brake(BrakeType::COAST);

    front_wings.set(front_wings_req);
    hang1.set(hang_req);
    hang2.set(hang_req);
    left_wing.set(left_wing_req || wings_req || master_left_wing_req);
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
    left_wing_req = 0;
    right_wing_req = 0;
    hang_req = Button::b_pressed;
    if (Digital::pressing(Button::UP)) {
      master_cata_req = 0;
      master_left_wing_req = 0;
    }
    pros::Task::delay_until(&nw, 5);
  }
}

bool cata_req = 0, front_wings_req = 0, wings_req = 0, hang_req = 0,
     left_wing_req = 0, right_wing_req = 0, intake_req = 0, intake_rev_req = 0,
     accel_off = 0, master_cata_req = 0, master_left_wing_req = 0;
float intake_speed = 127;

} // namespace sys_task