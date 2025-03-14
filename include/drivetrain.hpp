#pragma once
#include "globals.hpp"


//Function definitions
void driveTrain(char dirveScheme, bool isCurved, bool pneumaticsState);
void tankDrive(bool pneumaticsState);       //No sensitivity curve, linear
void tankDriveCubic(bool pneumaticsState);  //Sensitivty curve, cubic
void arcadeDrive();
void splitDrive();
int cubicCurve(int controllerInput);
void checkCurveInput();
void driveTrainMove(double dist, int velocity);
void driveTrainTurn(double theta, int velocity);
