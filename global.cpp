#include "main.h"

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Motor frontleft(-4);
pros::Motor frontright(5);
pros::Motor midleft(-2);
pros::Motor midright(8);
pros::Motor backleft(-1);
pros::Motor backright(9);


pros::Motor intake(14);
pros::Motor conveyor(19);
pros::Imu inertial(15);
pros::adi::DigitalOut piston('H');

