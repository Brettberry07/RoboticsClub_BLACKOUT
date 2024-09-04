#pragma once
#include "main.h"
#include "drivetrain.hpp"

//This is the global include, includes the main.h and all other bheader filesn I need to worry about

//Extern the global variables I definend in globals.cpp
extern pros::MotorGroup leftChassis;
extern pros::MotorGroup rightChassis;
extern int gearRatio;
extern bool isCurved;
extern pros::Controller master;