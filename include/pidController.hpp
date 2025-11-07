#pragma once
#include "globals.hpp"

// ----------------------------------------------------------------------------
// PID configuration and helpers
// ----------------------------------------------------------------------------

struct PIDConstants{
    double kP;
    double kI;
    double kD;
    double error = 0;
    double prevError = 0;
    double integral = 0;
    double derivative = 0;
    uint8_t timeOut = 8;
    // Symmetric integral bounds to avoid bias.
    double low = -12000, high = 12000;
    
};
// Function definitions
void linearPID(double target);
void angularPID(double target);

double getLinearError(double target, double leftTicks, double rightTicks);
double getAngularError(double target, double leftTicks, double rightTicks);

void updateOdom(double leftTicks, double rightTicks);
double degToRad(double deg);
double radToDeg(double rad);
