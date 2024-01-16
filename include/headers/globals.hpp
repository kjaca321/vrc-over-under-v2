#pragma once
#include "main.h"

extern Driver robot;
extern DigitalSystem front_wings, hang1, hang2, left_wing, right_wing;
extern pros::Motor cata, intake;

namespace auton {

extern uint32_t max_time;
extern unsigned int selected_auton, num_autons;

}; // namespace auton

namespace sys_task {

void run_systems(void);
void run_input(void);

extern bool cata_req, front_wings_req, wings_req, hang_req, left_wing_req,
    right_wing_req, intake_req, intake_rev_req, accel_off;

}; // namespace sys_task