#pragma once
#include "globals.hpp"

//function definitions
void linearPID(double target);
void angularPid(double target);
double getLinearError(double target, double leftTicks, double rightTicks);
double getAngularError(double target, double leftTicks, double rightTicks);
double normalizeAngle(double angle);
void updateOdom(double leftTicks, double rightTicks);
