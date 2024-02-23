#pragma once
#include "main.h"

#define run_driver_sequence()                                                  \
  pros::Task button_toggle(Digital::toggle_thread);                            \
  pros::Task driver(system_threads::drive_chassis);                            \
  pros::Task sys(system_threads::run_sys);                                     \
  pros::Task in(system_threads::run_in);                                       \
  pros::Task odom(system_threads::odometry);

#define kill_driver_sequence()                                                 \
  button_toggle.remove();                                                      \
  driver.remove();                                                             \
  sys.remove();                                                                \
  in.remove();                                                                 \
  odom.remove();

#define run_auton_sequence()                                                   \
  pros::Task odom(system_threads::odometry);                                   \
  pros::Task sys(system_threads::run_sys);

#define kill_auton_sequence()                                                  \
  odom.remove();                                                               \
  sys.remove();

#define run_master_auton()                                                     \
  run_auton_sequence();                                                        \
  auton::run_auton();                                                          \
  kill_auton_sequence();

namespace system_threads {

inline void drive_chassis(void *param) { robot.control(); }
inline void odometry(void *param) { robot.run_tracker(); }
inline void run_sys(void *param) { sys_task::run_systems(); }
inline void run_in(void *param) { sys_task::run_input(); }

}; // namespace system_threads