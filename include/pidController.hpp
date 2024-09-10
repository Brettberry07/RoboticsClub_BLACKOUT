#pragma once
#include "globals.hpp"

//function definitions
void linearPid();
void angularPid();
double getLinearError(double target);
double getAngularError(double target);
double normalizeAngle(double angle);

