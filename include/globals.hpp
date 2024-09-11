#pragma once
#include "main.h"
#include "drivetrain.hpp"
#include "pidController.hpp"

//This is the global include, includes the main.h and all other bheader filesn I need to worry about

//Extern the global variables I definend in globals.cpp
extern pros::MotorGroup leftChassis;
extern pros::MotorGroup rightChassis;
extern pros::IMU imuSensor;
extern double gearRatio;
extern bool isCurved;
extern bool driveOrIntake;
extern pros::Controller master;