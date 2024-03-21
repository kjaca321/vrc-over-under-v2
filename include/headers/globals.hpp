#pragma once
#include "main.h"

extern Driver robot;
extern DigitalSystem front_wings, hang1, hang2, left_wing, right_wing;
extern pros::Motor cata, cata2, intake, intake2;
extern pros::ADIDigitalIn auton_select;
extern pros::ADIDigitalIn auton_calibrate;

namespace auton {

void rush_6b(void);
void safe_6b(void);
void safe_4b(void);
void safe_3b_touch(void);
void safe_close(void);
void rush_close(void);
void rush_close_push(void);
void skills(void);
void skills_start(void);
void run_selection(void);
void run_auton(void);

extern uint32_t max_time;
extern unsigned int selected_auton, num_autons;
extern float MAX_SPEED, MAX_ACCEL;

}; // namespace auton

namespace sys_task {

void run_systems(void);
void run_input(void);

extern bool cata_req, front_wings_req, wings_req, hang_req, left_wing_req,
    right_wing_req, intake_req, intake_rev_req, accel_off, master_cata_req,
    master_left_wing_req;
extern float intake_speed;

}; // namespace sys_task