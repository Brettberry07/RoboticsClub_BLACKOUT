#pragma once
#include "tests.hpp"
#include "main.h"
#include "drivetrain_oop.hpp"
#include "pidController.hpp"
#include "pneumatics.hpp"
#include "intake.hpp"
#include "autonomous.hpp"
#include "liblvgl/lvgl.h"

// Global includes: aggregates headers used throughout the project.
// Extern declarations mirror definitions in globals.cpp.

// Drivetrain variables
extern pros::MotorGroup leftChassis;
extern pros::MotorGroup rightChassis;
extern pros::MotorGroup driveTrainMotors;
extern bool isCurved;

// Drift compensation: multiplier for left side motors to fix mechanical drift
extern double DRIFT_COMPENSATION;

// Turn sensitivity: multiplier for turning input to adjust turn speed
extern double TURN_SENSITIVITY;

// Used for drivetrain and autonomous
extern const double wheelRadius; // Inches
extern const double distPerTick;
extern const double distOneTick;
extern const double wheelBase;
extern double gearRatio;

// Tracking wheel constants (for rotation sensor)
extern const double trackingWheelDiameter;  // 2 inches
extern const double trackingWheelCircumference;  // π * diameter

// Intake motors (three-stage system)
extern pros::Motor lowIntakeMotor;
extern pros::Motor midIntakeMotor;
extern pros::Motor highIntakeMotor;

// Port sensors (tri-ports)
extern pros::IMU imuSensor;
extern pros::Rotation rotationSensor;  // Rotation sensor on port 1 for tracking
extern pros::adi::Port driveIntakePin;
extern pros::adi::Port clampPin;

// Pneumatics states
extern bool clampPneumaticsState;

// Autonomous selector variables
extern bool autonSelected;
extern char autonID;

// Controller
extern pros::Controller master;

// Screen
extern pros::screen_touch_status_s_t status;

// Odometry variables
extern double globalHeading;
extern double globalPos[2]; // Holds X and Y values