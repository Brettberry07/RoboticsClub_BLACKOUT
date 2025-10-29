#pragma once
#include "globals.hpp"

// ----------------------------------------------------------------------------
// Drivetrain interface
// ----------------------------------------------------------------------------
// These declarations expose multiple drive schemes and helpers used across
// operator control and autonomous routines. Comments are standardized for
// readability while preserving all original intent and information.

// Dispatch by scheme: 't' Tank, 's' Split, 'a' Arcade. isCurved toggles cubic input shaping.
void driveTrain(char driveScheme, bool isCurved);

// Tank drive (linear response).
void tankDrive();

// Tank drive with cubic sensitivity curve (precision control).
void tankDriveCubic();

// One-stick arcade control (power = left Y, turn = left X).
void arcadeDrive();

// Split arcade control (power = left Y, turn = right X).
void splitDrive();

// Cubic input shaper helper.
int cubicCurve(int controllerInput);

// Toggle curved/linear input on button press (rising edge).
void checkCurveInput();

// Autonomous helpers: move a distance (inches) or turn by angle (degrees).
void driveTrainMove(double dist, int velocity);
void driveTrainTurn(double theta, int velocity);
