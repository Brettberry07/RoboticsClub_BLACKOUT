#pragma once
#include "globals.hpp"

struct PIDconstants{
    double kP;
    double kI;
    double kD;
    double error = 0;
    double prevError = 0;
    double integral = 0;
    double derivative = 0;
    int timeOut = 50000; // Should make adaptive, explanation in pidController.cpp
};
//function definitions
void linearPID(double target);
void angularPid(double target);
double getLinearError(double target, double leftTicks, double rightTicks);
double getAngularError(double target, double leftTicks, double rightTicks);
double normalizeAngle(double angle);
void updateOdom(double leftTicks, double rightTicks);
