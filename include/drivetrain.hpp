#pragma once
#include "globals.hpp"


//Function definitions
void driveTrain(char dirveScheme, bool isCurved);
void tankDrive();       //No sensitivity curve, linear
void tankDriveCubic();  //Sensitivty curve, cubic
void arcadeDriveTwo();
void arcadeDriveOne();
int cubicCurve(int controllerInput);
void checkCurveInput();