#include "globals.hpp"

//Define all my global definitions like the motors and controllers
pros::MotorGroup leftChassis({-1,-2});
pros::MotorGroup rightChassis({9,10});
int gearRatio = 1;
bool isCurved = false;
pros::Controller master(pros::E_CONTROLLER_MASTER);