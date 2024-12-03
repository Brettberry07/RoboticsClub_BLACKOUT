#pragma once
#include "globals.hpp"

struct PIDConstants{
    double kP;
    double kI;
    double kD;
    double error = 0;
    double prevError = 0;
    double integral = 0;
    double derivative = 0;
    int timeOut = 50000; // Should make adaptive, explanation in pidController.cpp
    double low = 0, high = 12000;
    
};
//function definitions
void linearPID(double target);
void angularPID(double target);
double getLinearError(double target, double leftTicks, double rightTicks);
double getAngularError(double target, double leftTicks, double rightTicks);
void updateOdom(double leftTicks, double rightTicks);
