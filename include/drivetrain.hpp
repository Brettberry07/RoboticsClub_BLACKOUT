#pragma once
#include "globals.hpp"


//Function definitions
void driveTrain(char driveScheme, bool isCurved);
void tankDrive();       // No sensitivity curve, linear
void tankDriveCubic();  // Sensitivity curve, cubic
void arcadeDrive();
void splitDrive();
int cubicCurve(int controllerInput);
void checkCurveInput();
void driveTrainMove(double dist, int velocity);
void driveTrainTurn(double theta, int velocity);
