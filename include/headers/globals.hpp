#pragma once
#include "main.h"

extern Driver robot;
extern DigitalSystem front_wings, hang1, hang2, left_wing, right_wing;
extern pros::Motor cata, intake;
extern pros::ADIDigitalIn auton_select;
extern pros::ADIDigitalIn auton_calibrate;

namespace auton {

void safe_6b(void);
void run_selection(void);
void run_auton(void);

extern uint32_t max_time;
extern unsigned int selected_auton, num_autons;

}; // namespace auton

namespace sys_task {

void run_systems(void);
void run_input(void);

extern bool cata_req, front_wings_req, wings_req, hang_req, left_wing_req,
    right_wing_req, intake_req, intake_rev_req, accel_off;
extern float intake_speed;

}; // namespace sys_task