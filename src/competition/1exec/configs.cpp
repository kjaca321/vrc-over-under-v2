#include "main.h"

Driver robot(std::vector<int>{-11, -13, -14}, std::vector<int>{17, 20, 19},
             3.25, 450, 15, 11.0);

DigitalSystem front_wings('D');

DigitalSystem hang1('C'), hang2('F');

DigitalSystem left_wing('H'), right_wing('G');

pros::Motor cata(16);
pros::Motor cata2(-1);
pros::Motor intake(5);
pros::Motor intake2(-7);

pros::ADIDigitalIn auton_select('B');
pros::ADIDigitalIn auton_calibrate('E');