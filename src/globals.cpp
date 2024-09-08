#include "globals.hpp"

//Define all my global definitions like the motors and controllers
pros::MotorGroup leftChassis({-1,-2});
pros::MotorGroup rightChassis({9,10});
pros::IMU imuSensor(3);
double gearRatio = 0.6;   //motor 36: Wheel: 60 or 1.875,idk
bool isCurved = false;
pros::Controller master(pros::E_CONTROLLER_MASTER);