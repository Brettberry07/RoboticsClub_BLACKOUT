#include "globals.hpp"
#include "robot.hpp"
#include <cmath>

/**
 * PID Controller Pseudocode (Original Notes)
 * -----------------------------------------
 *
 * linPID():
 *   proportion = currentPos - desiredPos
 *   integral  += error
 *   derivative = error - prevError
 *
 *   kP = proportional gain
 *   kI = integral gain           // These constants tune the PID
 *   kD = derivative gain
 *
 *   Assume we know the target distance (solved via odometry in practice).
 *
 *   while (error > someThreshold):           // Can't reach perfection; choose a near value
 *     proportion = currentPos - desiredPos   // How far we are
 *     integral  += error                     // Accumulated error
 *     derivative = error - prevError         // Rate of approach to goal
 *
 *     power = kP*proportion + kI*integral + kD*derivative
 *
 *     // Go straight
 *     RightMotor.move(power)
 *     LeftMotor.move(power)
 *
 *     // Or turn
 *     RightMotor.move(power)
 *     LeftMotor.move(-power)
 *
 *   loop:
 *     error = setpoint - measured_value
 *     proportional = error
 *     integral += error * dt
 *     derivative = (error - previous_error) / dt
 *     output = Kp*proportional + Ki*integral + Kd*derivative
 *     previous_error = error
 *     wait(dt)
 *
 *   // Stop motors (should already be near zero)
 *   rightMotor.move(0)
 *   leftMotor.move(0)
 */

// 2*Radius*PI/gearRatio/ticksPerRevolution

//ticks per revolution
//Blue: 1800 
//Green: 900
//Red: 300

// These constants determine how the PID reacts to error:
// - P responds to current error (main driving force).
// - I compensates for accumulated error the P term can't eliminate.
// - D damps overshoot by reacting to the rate of change of error.
// Tuned for voltage control (millivolts, max ±12000)
PIDConstants linPID = {50, 1, 1};  // Increased kP for voltage control, reduced kI, increased kD for damping
PIDConstants angPID = {275, 150, 50}; // good, but could be better


// Cameron: consider making timeout adaptive based on distance/angle magnitude.
// Harder-to-reach goals should have a higher timeout.
// const int timeOut = 50000;

/**
 * Linear PID Control
 * ------------------
 * Implements a PID controller for linear motion.
 *
 * @param target Target linear position (inches).
 */

void linearPID(double target) {
    // Reset drivetrain encoders
    getRobot().drivetrain.tare();

    // Reset PID state
    linPID.error = 0;
    linPID.prevError = 0;
    linPID.integral = 0;

    uint32_t previousTime = pros::millis();
    double totalTime = 0;
    bool firstIteration = true;

    while (true) {
        firstIteration = false;
        
    // Get current positions from both sides.
        double leftTicks, rightTicks;
        getRobot().drivetrain.getPosition(leftTicks, rightTicks);


        // Calculate error as the difference between target and the current average position.
        // (Assumes your getLinearError function returns target - currentPosition)
        linPID.error = getLinearError(target, leftTicks, rightTicks);
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Error: %f", linPID.error);

    // Get time delta in seconds.
        uint32_t currentTime = pros::millis();
        double dt = (currentTime - previousTime) / 1000.0;
        totalTime += dt;

        // Check for timeout (absolute time based on millis).
        if ((totalTime) >= linPID.timeOut) {
            pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Time Out, Time reached: %f", linPID.timeOut);
            break;
        } 

        // Check if error is within acceptable range BEFORE computing power
        if (fabs(linPID.error) < 1.0) {
            pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Min range met. Range: %f", linPID.error);
            break;
        }

        // Compute derivative (change in error over time), skip on first iteration to avoid spike
    if (firstIteration || dt < 0.001) {
        linPID.derivative = 0;
        firstIteration = false;
    } else {
        linPID.derivative = (linPID.error - linPID.prevError) / dt;
    }
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Derivative: %f", linPID.derivative);

        // Accumulate the integral term using dt so the gain is time‐scaled.
    linPID.integral += linPID.error * dt;
    // Clamp the integral to prevent windup.
    if (linPID.integral > linPID.high) linPID.integral = linPID.high;
    if (linPID.integral < linPID.low)  linPID.integral = linPID.low;
        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Integral: %f", linPID.integral);

        // Calculate PID output.
    int32_t power = (linPID.kP * linPID.error) + 
            (linPID.kI * linPID.integral) + 
            (linPID.kD * linPID.derivative);
    
    // Apply minimum power threshold to overcome static friction
    if (power > 0 && power < 1500) power = 1500;
    if (power < 0 && power > -1500) power = -1500;
    
    // Clamp output to motor voltage range.
    if (power > 12000) power = 12000;
    if (power < -12000) power = -12000;
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Power: %d", power);

    // Apply symmetric voltage to both sides
    getRobot().drivetrain.setVoltage(power, power);

        // Update previous error and time for next loop
        linPID.prevError = linPID.error;
        previousTime = currentTime;

        pros::delay(10);
    }

    // Once the loop is done, brake the chassis.
    getRobot().drivetrain.brake();

    pros::delay(100); // Allow time for motors to stop completely.
}




// Cameron: made a struct for this; Brett: variables converted to the struct.

/**
 * Angular PID Control
 * -------------------
 * Implements a PID controller for angular motion.
 *
 * @param target Target heading (degrees).
 */
void angularPID(double target) {
    int32_t power = 0;
    double leftTicks = 0, rightTicks = 0;

    // Reset chassis positions
    getRobot().drivetrain.tare();
    
    // Reset PID state
    angPID.error = 0;
    angPID.prevError = 0;
    angPID.integral = 0;

    uint32_t previousTime = pros::millis();
    double totalTime = 0;
    bool firstIteration = true;

    while (true) {
        getRobot().drivetrain.getPosition(leftTicks, rightTicks);

        
        angPID.error = getAngularError(target, leftTicks, rightTicks);
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Error: %f", angPID.error);

        uint32_t currentTime = pros::millis();
    double dt = (currentTime - previousTime) / 1000.0; // dt in seconds.
        totalTime += dt;

        // Check for timeout (absolute time based on millis).
        if ((totalTime) >= angPID.timeOut) {
            pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Time Out, Time reached: %f", angPID.timeOut);
            break;
        } 

        // Exit if error is within acceptable threshold BEFORE computing power
        if (fabs(angPID.error) < 1.0) {
            pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Min range met. Range: %f", angPID.error);
            break;
        }

        // Use dt in derivative and integral calculations, skip derivative on first iteration
    if (firstIteration || dt < 0.001) {
        angPID.derivative = 0;
        firstIteration = false;
    } else {
        angPID.derivative = (angPID.error - angPID.prevError) / dt;
    }
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Derivative: %f", angPID.derivative);

    angPID.integral += angPID.error * dt;
    if (angPID.integral > angPID.high) angPID.integral = angPID.high;
    if (angPID.integral < angPID.low)  angPID.integral = angPID.low;
        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Integral: %f", angPID.integral);

        // Compute control signal (power).
    power = (angPID.kP * angPID.error) + 
        (angPID.kI * angPID.integral) + 
        (angPID.kD * angPID.derivative);
    
    // Apply minimum power threshold to overcome static friction
    if (power > 0 && power < 1500) power = 1500;
    if (power < 0 && power > -1500) power = -1500;
    
    if (power > 12000) power = 12000;
    if (power < -12000) power = -12000;
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Power: %d", power);

    // Apply motor commands (differential drive: one side reversed)
    getRobot().drivetrain.setVoltage(power, -power);

        angPID.prevError = angPID.error;
        previousTime = currentTime;

        pros::delay(10);
    }

    getRobot().drivetrain.brake();
    pros::delay(100); // Allow time for motors to stop completely.
}



/**
 * Linear Error Calculation
 * ------------------------
 * Calculates linear error between a target value and the current position.
 *
 * @param target Target distance (inches).
 * @param leftTicks Left encoder ticks.
 * @param rightTicks Right encoder ticks.
 * @return Linear error (inches).
 */
double getLinearError(double target, double leftTicks, double rightTicks) {
    // Get the average between the two sides.
    updateOdom(leftTicks, rightTicks);
    double temp = ((distOneTick * rightTicks) + (distOneTick * leftTicks))/2;
    pros::screen::print(pros::E_TEXT_MEDIUM, 5, "pos: %f", temp);
    return target - temp;

}

// /**
//  * Calculates the angular error between the target angle and the current global heading.
//  *
//  * @param target The target angle in radians.
//  * @param leftTicks The number of left wheel encoder ticks.
//  * @param rightTicks The number of right wheel encoder ticks.
//  * @return The angular error in radians.
//  */
// double getAngularError(double target, double leftTicks, double rightTicks) {
//     updateOdom(leftTicks, rightTicks);
//     // Compute error and wrap within [-π, π] (Radians)
//     double error = target - globalHeading;
//     return error;
// }

double getAngularError(double target, double leftTicks, double rightTicks) {
    updateOdom(leftTicks, rightTicks);
    double error = target - globalHeading;
    // Wrap error into the range [-180, 180].
    while (error > 180)  error -= 360;
    while (error < -180) error += 360;
    return error;
}


// Converts degrees to radians.
/**
 * Degrees to Radians
 * ------------------
 * @param deg Angle in degrees.
 * @return Angle in radians.
 */
double degToRad(double deg) {
    return deg * (M_PI / 180);
}

// Converts radians to degrees.
/**
 * Radians to Degrees
 * ------------------
 * @param rad Angle in radians.
 * @return Angle in degrees.
 */
double radToDeg(double rad) {
    return rad * (180 / M_PI);
}

/**
 * Update Odometry
 * ---------------
 * Integrates motion based on encoder delta ticks and IMU heading.
 *
 * - Converts delta ticks to distances for each side.
 * - Computes average distance for this cycle and updates global X/Y using IMU heading.
 * - Prints current global heading (degrees) to the screen.
 *
 * @param leftTicks  Left wheel encoder ticks (absolute reading).
 * @param rightTicks Right wheel encoder ticks (absolute reading).
 */
void updateOdom(double leftTicks, double rightTicks) {
    // Delegate to OOP odometry; it mirrors into globalPos/globalHeading for compatibility.
    getRobot().odometry.update(leftTicks, rightTicks);
}

/*
Pseudocode for updateOdom (Original Notes)
-----------------------------------------

updateOdom(leftTicks, rightTicks){
    distLeft  = leftTicks  * distOneTick
    distRight = rightTicks * distOneTick

    averageDist = (distLeft + distRight) / 2

    // Change in heading with respect to wheelBase
    deltaTheta = (distRight - distLeft) / wheelBase

    deltaTheta = int(deltaTheta) % 360  // constrain to [0, 360)

    globalHeading += deltaTheta  // update heading

    // Update X/Y on the coordinate plane
    globalPos[x] = globalPos[x] + averageDist * cos(globalHeading)
    globalPos[y] = globalPos[y] + averageDist * sin(globalHeading)

    print("Updated global heading, global heading: ", globalHeading)
}
*/