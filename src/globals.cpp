#include "globals.hpp"

pros::MotorGroup leftChassis({-1,-2});
pros::MotorGroup rightChassis({9,10});
int gearRatio = 1;
pros::Controller master(pros::E_CONTROLLER_MASTER);