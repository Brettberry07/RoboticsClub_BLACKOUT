#pragma once
#include "main.h"
#include "drivetrain.hpp"
#include "pidController.hpp"
#include "pneumatics.hpp"
#include "intake.hpp"
#include "autonomous.hpp"
#include "screen.hpp"

//This is the global include, includes the main.h and all other bheader filesn I need to worry about
//Extern the global variables I definend in globals.cpp

//drivetrain variables
extern pros::MotorGroup leftChassis;
extern pros::MotorGroup pneumaticsLeftChassis;
extern pros::MotorGroup rightChassis;
extern pros::MotorGroup pneumaticsRightChassis;
extern pros::MotorGroup driveTrainMotors;
extern bool isCurved;

//used for drivetrain and autonomous
extern const uint8_t wheelRadius;
extern const double distPerTick;
extern const double distOneTick;
extern const double wheelBase;
extern double gearRatio;

//intake variables
extern pros::MotorGroup intakeMotors;

//port sensors (tri-ports)
extern pros::IMU imuSensor;
extern pros::adi::Port driveIntakePin;
extern pros::adi::Port clampPin;

//pneumatics states
extern bool driveOrIntakeState;
extern bool clampPneumaticsState;

//auton selector variables
extern bool autonSelected;
extern char autonID;

//controller
extern pros::Controller master;

//screen
extern pros::screen_touch_status_s_t status;

//odom variables
extern double globalHeading;
extern double globalPos[2]; //holds x and y values

void straightAuton();