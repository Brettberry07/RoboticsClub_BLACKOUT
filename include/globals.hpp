#pragma once
#include "main.h"
#include "drivetrain.hpp"
#include "pidController.hpp"
#include "pneumatics.hpp"
#include "intake.hpp"

//This is the global include, includes the main.h and all other bheader filesn I need to worry about

//Extern the global variables I definend in globals.cpp
extern pros::MotorGroup leftChassis;
extern pros::MotorGroup pneumaticsLeftChassis;
extern pros::MotorGroup rightChassis;
extern pros::MotorGroup pneumaticsRightChassis;
extern pros::MotorGroup intakeMotors;
extern pros::IMU imuSensor;
extern pros::adi::Port driveIntakePin;
extern pros::adi::Port clampPin;

extern double gearRatio;
extern bool isCurved;
extern bool driveTrainPneumaticsState;
extern bool clampPneumaticsState;
extern pros::Controller master;