#include "globals.hpp"
#include "robot.hpp"
#include <cmath>

/*
================================================================================
PID CONTROLLER - PSEUDOCODE
================================================================================

PURPOSE: Closed-loop control for accurate linear (straight) and angular (turn)
         movements using PID (Proportional-Integral-Derivative) algorithm

SENSORS:
- Linear: Rotation sensor on port 1 (2-inch tracking wheel)
- Angular: IMU sensor on port 5 (gyroscope)

CONTROL OUTPUT: Motor voltage in millivolts [-12000, 12000]

================================================================================

PID TUNING CONSTANTS
--------------------

Linear PID (linPID):
    kP = 500   // Proportional: main driving force (500 × 24" = 12000mV)
    kI = 5     // Integral: eliminates steady-state error
    kD = 80    // Derivative: dampens oscillation and overshoot
    timeout = 3 seconds

Angular PID (angPID):
    kP = 275   // Proportional: turn rate
    kI = 150   // Integral: corrects persistent angle error
    kD = 50    // Derivative: reduces overshoot in turns
    timeout = 3 seconds

================================================================================

LINEAR PID - STRAIGHT LINE MOVEMENT
------------------------------------

void linearPID(target):
    // Move robot straight to target distance using tracking wheel
    
    INITIALIZATION:
        reset rotation sensor position to 0
        reset PID state (error, prevError, integral) to 0
        start timer
        set firstIteration flag to true
        delay 10ms for clean first reading
    
    CONTROL LOOP (runs at 100Hz):
        READ SENSOR:
            get sensor angle in centidegrees
            convert to degrees (centidegrees / 100)
            calculate distance = (angle / 360) × wheel circumference
            // 2-inch wheel circumference = π × 2 ≈ 6.283 inches
        
        CALCULATE ERROR:
            error = target - distance traveled
            // Positive error = need to go forward
            // Negative error = need to go backward
        
        CHECK EXIT CONDITIONS:
            if time >= 3 seconds:
                timeout, break loop
            if |error| < 0.5 inches:
                target reached, break loop
        
        COMPUTE DERIVATIVE:
            if first iteration or dt < 0.001:
                derivative = 0  // prevent spike
            else:
                derivative = (error - prevError) / dt
        
        ACCUMULATE INTEGRAL:
            integral += error × dt
            clamp integral to [-800, 800]  // prevent windup
        
        CALCULATE OUTPUT:
            power = (500 × error) + (5 × integral) + (80 × derivative)
            
            if |error| > 3 inches AND |power| < 1500mV:
                apply minimum 1500mV  // overcome static friction
            
            clamp power to [-12000, 12000] millivolts
        
        APPLY POWER:
            set both left and right motors to power
        
        UPDATE STATE:
            prevError = error
            delay 10ms
    
    CLEANUP:
        brake motors
        delay 100ms for complete stop

EXAMPLE: linearPID(24)
    - Starts at max power (12000mV) when 24" away
    - Gradually reduces as distance closes
    - Arrives smoothly within 0.5" of target

================================================================================

ANGULAR PID - TURNING IN PLACE
-------------------------------

void angularPID(target):
    // Turn robot to target heading using IMU
    
    INITIALIZATION:
        reset IMU heading to 0 degrees
        reset PID state (error, prevError, integral) to 0
        start timer
        set firstIteration flag to true
    
    CONTROL LOOP (runs at 100Hz):
        READ SENSOR:
            get current heading from IMU (0-360 degrees)
        
        CALCULATE ERROR:
            error = target - currentHeading
            
            normalize to shortest path [-180, 180]:
                while error > 180: error -= 360
                while error < -180: error += 360
            // Example: target=10°, current=350° → error=20° (turn right)
        
        CHECK EXIT CONDITIONS:
            if time >= 3 seconds:
                timeout, break loop
            if |error| < 1.0 degrees:
                target reached, break loop
        
        COMPUTE DERIVATIVE:
            if first iteration or dt < 0.001:
                derivative = 0  // prevent spike
            else:
                derivative = (error - prevError) / dt
        
        ACCUMULATE INTEGRAL:
            integral += error × dt
            clamp integral to [-12000, 12000]  // prevent windup
        
        CALCULATE OUTPUT:
            power = (275 × error) + (150 × integral) + (50 × derivative)
            
            if |power| < 1500mV:
                apply minimum 1500mV  // overcome static friction
            
            clamp power to [-12000, 12000] millivolts
        
        APPLY POWER:
            left motors: +power (forward)
            right motors: -power (backward)
            // Differential steering for in-place turn
        
        UPDATE STATE:
            prevError = error
            delay 10ms
    
    CLEANUP:
        brake motors
        delay 100ms for complete stop

EXAMPLE: angularPID(90)
    - IMU reads 0°, target 90°
    - Error = 90°, applies strong right turn
    - Reduces power as angle approaches 90°
    - Stops within 1° of target

================================================================================

HELPER FUNCTIONS
----------------

double getLinearError(target, leftTicks, rightTicks):
    // Calculate linear distance error (legacy, not used with rotation sensor)
    update odometry with encoder ticks
    average = (leftTicks + rightTicks) / 2 × distOneTick
    return target - average

double getAngularError(target, leftTicks, rightTicks):
    // Calculate angular error (legacy, not used with IMU)
    update odometry with encoder ticks
    error = target - globalHeading
    normalize to [-180, 180]
    return error

void updateOdom(leftTicks, rightTicks):
    // Update global odometry position
    delegate to getRobot().odometry.update(leftTicks, rightTicks)
    // Mirrors position to globalPos[] and globalHeading

double degToRad(deg):
    return deg × (π / 180)

double radToDeg(rad):
    return rad × (180 / π)

================================================================================

PID CONTROL THEORY
------------------

PROPORTIONAL (P):
    - Reacts to current error magnitude
    - Larger error → more power
    - Problem: can never quite reach zero (steady-state error)

INTEGRAL (I):
    - Accumulates error over time
    - Eliminates steady-state error
    - Problem: can cause overshoot (integral windup)
    - Solution: clamp integral to reasonable bounds

DERIVATIVE (D):
    - Reacts to rate of change of error
    - Predicts future error trend
    - Dampens oscillation and overshoot
    - Problem: sensitive to noise
    - Solution: skip first iteration, check dt validity

COMBINED (PID):
    output = kP×error + kI×∫error dt + kD×(d error/dt)
    
    - P gets you close quickly
    - I eliminates remaining error
    - D prevents overshooting

================================================================================
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
// For 24 inch target: kP alone gives 24*500 = 12000mV (full power at max error)
PIDConstants linPID = {320, 150, 55};  // kP for responsiveness, kI for steady-state, kD for damping
PIDConstants angPID = {275, 150, 50}; // good, but could be better


// TODO: consider making timeout adaptive based on distance/angle magnitude.
// Harder-to-reach goals should have a higher timeout.
// const int timeOut = 50000;

/**
 * Linear PID Control
 * ------------------
 * Implements a PID controller for linear motion using rotation sensor.
 *
 * @param target Target linear position (inches).
 */

void linearPID(double target) {
    // Reset rotation sensor to zero
    rotationSensor.reset_position();
    target--;  // Minor adjustment to account for overshoot

    // Reset PID state
    linPID.error = 0;
    linPID.prevError = 0;
    linPID.integral = 0;

    uint32_t previousTime = pros::millis();
    double totalTime = 0;
    bool firstIteration = true;
    
    // Store initial time for derivative calculation
    pros::delay(10);  // Small delay to ensure clean first measurement

    while (true) {
        
        // Get current position from rotation sensor (returns centidegrees)
        int32_t sensorCentidegrees = rotationSensor.get_position();
        double sensorAngle = sensorCentidegrees / 100.0;  // Convert to degrees
        
        // Calculate distance traveled: (angle / 360) * circumference
        // For forward motion, positive angle = positive distance
        double distanceTraveled = (sensorAngle / 360.0) * trackingWheelCircumference;

        // Calculate error as the difference between target and current position
        linPID.error = target - distanceTraveled;
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Err: %.2f in, Dist: %.2f", linPID.error, distanceTraveled);

    // Get time delta in seconds.
        uint32_t currentTime = pros::millis();
        double dt = (currentTime - previousTime) / 1000.0;
        totalTime += dt;

        // Check for timeout (absolute time based on millis).
        if (totalTime >= linPID.timeOut) {
            pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Timeout at %.2f sec", totalTime);
            break;
        } 

        // Check if error is within acceptable range BEFORE computing power
        // Tighter tolerance for better accuracy
        if (fabs(linPID.error) < 0.5) {
            pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Target reached! Err: %.2f", linPID.error);
            break;
        }

        // Compute derivative (change in error over time), skip on first iteration to avoid spike
        if (firstIteration || dt < 0.001) {
            linPID.derivative = 0;
            firstIteration = false;
        } else {
            linPID.derivative = (linPID.error - linPID.prevError) / dt;
        }
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "D: %.1f", linPID.derivative);

        // Accumulate the integral term using dt so the gain is time‐scaled.
        linPID.integral += linPID.error * dt;
        // Clamp the integral to prevent windup (limit integral contribution to ±4000mV)
        double integralMax = 800.0;  // 800 * 5 (kI) = 4000mV max contribution
        if (linPID.integral > integralMax) linPID.integral = integralMax;
        if (linPID.integral < -integralMax) linPID.integral = -integralMax;
        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "I: %.1f", linPID.integral);

        // Calculate PID output in millivolts
        int32_t power = (linPID.kP * linPID.error) + 
                (linPID.kI * linPID.integral) + 
                (linPID.kD * linPID.derivative);
    
        // Apply minimum power threshold to overcome static friction (only when far from target)
        if (fabs(linPID.error) > 3.0) {  // Only apply minimum when >3 inches away
            if (power > 0 && power < 1500) power = 1500;
            if (power < 0 && power > -1500) power = -1500;
        }
    
        // Clamp output to motor voltage range.
        if (power > 12000) power = 12000;
        if (power < -12000) power = -12000;
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Pwr: %d mV", power);

        // Apply symmetric voltage to both sides
        getRobot().drivetrain.setVoltage(power, power);

        // Update previous error and time for next loop
        linPID.prevError = linPID.error;
        previousTime = currentTime;

        pros::delay(10);  // 100Hz control loop
    }

    // Once the loop is done, brake the chassis.
    getRobot().drivetrain.brake();

    pros::delay(100); // Allow time for motors to stop completely.
}




// Cameron: made a struct for this; Brett: variables converted to the struct.




// Cameron: made a struct for this; Brett: variables converted to the struct.

/**
 * Angular PID Control
 * -------------------
 * Implements a PID controller for angular motion using IMU.
 *
 * @param target Target heading (degrees).
 */
void angularPID(double target) {
    int32_t power = 0;

    // Reset IMU heading to 0
    imuSensor.tare_heading();
    
    // Reset PID state
    angPID.error = 0;
    angPID.prevError = 0;
    angPID.integral = 0;

    uint32_t previousTime = pros::millis();
    double totalTime = 0;
    bool firstIteration = true;

    while (true) {
        // Get current heading from IMU (0-360 degrees)
        double currentHeading = imuSensor.get_heading();
        
        // Calculate error between target and current heading
        angPID.error = target - currentHeading;
        
        // Normalize error to [-180, 180] for shortest path
        while (angPID.error > 180) angPID.error -= 360;
        while (angPID.error < -180) angPID.error += 360;
        
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Error: %.2f deg (IMU: %.1f)", angPID.error, currentHeading);

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