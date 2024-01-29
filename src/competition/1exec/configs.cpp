#include "main.h"

Driver robot(std::vector<int>{12, 13, 14}, std::vector<int>{-17, -18, -19},
             2.75, 600, 16, 8.7);

DigitalSystem front_wings('D');

DigitalSystem hang1('C'), hang2('F');

DigitalSystem left_wing('G'), right_wing('H');

pros::Motor cata(-20);
pros::Motor intake(15);

pros::ADIDigitalIn auton_select('B');
pros::ADIDigitalIn auton_calibrate('A');