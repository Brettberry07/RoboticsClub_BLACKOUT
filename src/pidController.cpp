#include "globals.hpp"
#include <cmath>

/*
Sudo code for now:

linPID(){
    proportion = currentPos - desiredPos
    integral += error
    derivative = error-prevError

    kP = constant for proportion
    kI = constant ofr integral          //These are how I tune the PID
    kD = constant for derivative


    Lets just say for this I know how far I want to move for
    simplicity, I'm assuming I would solve this through some
    sort of odometry thing

    while(error>someNumber){        //we can't reach perfection so choose a value close to it
        proportion = currentPos - desiredPos               //This is how far we are
        integral += error                                  //all errors
        derivative = error - prevError                     //How fast were approaching goal basically

        power = (proportion*kP)+(integral*kI)+(derivative*kD)

        //How I would go straight
        RightMotor.move(power)
        LeftMotor.move(power)
        
        //How I would turn
        RightMotor.move(power)      
        LeftMotor.move(-power)
    }

    loop{
        error = setpoint − measured_value
        proportional = error;
        integral = integral + error × dt
        derivative = (error - previous_error) / dt
        output = Kp × proportional + Ki × integral + Kd × derivative
        previous_error = error
        wait(dt)
    }

    rightMotor.move(0) //set the voltage for each motor to 0 so it stops moving, shoukd alreadty be close ot ir anyways
    leftMotor.move(0)
}
*/

// 2*Radius*PI/gearRatio/ticksPerRevolution

//ticks per revolution
//Blue: 1800 
//Green: 900
//Red: 300

// These constants bassically weigh how our PID system reacts to the current error
// P reacts to current overall error, main driving force so to speak 
// I controls the compensation for the total accumulated error (the error the P can't solve)
// D solves the problem of overeacting to the error, allows us to slow down as we rech target
PIDConstants linPID = {750, 100, 150};
PIDConstants angPID = {275, 150, 50}; // good, but could be better


// Cameron here, should make this adaptive based on distance or angular target magnetude 
// Harder to reach goals should have a higher timeout
// const int timeOut = 50000;

/**
 * @brief Implements a PID controller for linear control.
 * 
 * @param target The target value for the linear position.
 *
*/

void linearPID(double target) {
    // Reset motor positions before starting PID loop
    driveTrainMotors.tare_position();

    // Reset PID state
    linPID.error = 0;
    linPID.prevError = 0;
    linPID.integral = 0;

    uint32_t previousTime = pros::millis();
    double totalTime = 0;

    while (true) {
        
        // Get current positions from both sides
        double leftTicks = leftChassis.get_position();
        double rightTicks = rightChassis.get_position();


        // Calculate error as the difference between target and the current average position.
        // (Assumes your getLinearError function returns target - currentPosition)
        linPID.error = getLinearError(target, leftTicks, rightTicks);
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Error: %f", linPID.error);

        // Get time delta in seconds
        uint32_t currentTime = pros::millis();
        double dt = (currentTime - previousTime) / 1000.0;
        totalTime += dt;


        // Compute derivative (change in error over time)
    linPID.derivative = (linPID.error - linPID.prevError) / dt;
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Derivative: %f", linPID.derivative);

        // Accumulate the integral term, using dt so the gain is time‐scaled.
    linPID.integral += linPID.error * dt;
    // Clamp the integral to prevent windup
    if (linPID.integral > linPID.high) linPID.integral = linPID.high;
    if (linPID.integral < linPID.low)  linPID.integral = linPID.low;
        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Integral: %f", linPID.integral);

        // Calculate PID output
    int32_t power = (linPID.kP * linPID.error) + 
            (linPID.kI * linPID.integral) + 
            (linPID.kD * linPID.derivative);
    // Clamp output to motor voltage range
    if (power > 12000) power = 12000;
    if (power < -12000) power = -12000;
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Power: %d", power);

        // Check for timeout (using currentTime if you want an absolute time based on millis)
        if ((totalTime) >= linPID.timeOut) {
            pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Time Out, Time reached: %f", linPID.timeOut);
            break;
        } 

        // Check if error is within an acceptable range
        if (fabs(linPID.error) < 1) {
            pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Min range met. Range: %f", linPID.error);
            break;
        }

        // Move motors; if target is negative, error will be negative and so will the power
        driveTrainMotors.move_voltage(power);

        // Update previous error and time for next loop
        linPID.prevError = linPID.error;
        previousTime = currentTime;

        pros::delay(10);
    }

    // Once the loop is done, brake the chassis
    rightChassis.brake();
    leftChassis.brake();

    pros::delay(100); // Allow some time for motors to stop completely
}




// Cameron here, made a struct for this, haven't converted variables.
// Brett here, I've converted the variables to the struct

/**
 * @brief Implements a PID controller for angular control.
 * 
 * @param target The target value for the angular heading.
 */
void angularPID(double target) {
    int32_t power = 0;
    double leftTicks = 0, rightTicks = 0;

    // Reset chassis positions
    rightChassis.tare_position();
    leftChassis.tare_position();
    
    // Reset PID state
    angPID.error = 0;
    angPID.prevError = 0;
    angPID.integral = 0;

    uint32_t previousTime = pros::millis();
    double totalTime = 0;

    while (true) {
        leftTicks = leftChassis.get_position();
        rightTicks = rightChassis.get_position();

        
        angPID.error = getAngularError(target, leftTicks, rightTicks);
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Error: %f", angPID.error);

        uint32_t currentTime = pros::millis();
        double dt = (currentTime - previousTime) / 1000.0; // dt in seconds
        totalTime += dt;

        // Use dt in derivative and integral calculations
    angPID.derivative = (angPID.error - angPID.prevError) / dt;
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Derivative: %f", angPID.derivative);

    angPID.integral += angPID.error * dt;
    if (angPID.integral > angPID.high) angPID.integral = angPID.high;
    if (angPID.integral < angPID.low)  angPID.integral = angPID.low;
        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Integral: %f", angPID.integral);

        // Compute control signal (power)
    power = (angPID.kP * angPID.error) + 
        (angPID.kI * angPID.integral) + 
        (angPID.kD * angPID.derivative);
    if (power > 12000) power = 12000;
    if (power < -12000) power = -12000;
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Power: %d", power);

        // Check for timeout (using currentTime if you want an absolute time based on millis)
        if ((totalTime) >= angPID.timeOut) {
            pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Time Out, Time reached: %f", angPID.timeOut);
            break;
        } 

        // Exit if error is within acceptable threshold (1 degree)
        if (fabs(angPID.error) < 1) {
            pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Min range met. Range: %f", angPID.error);
            break;
        }

        // Apply motor commands (differential drive: one side reversed)
        rightChassis.move_voltage(-power);
        leftChassis.move_voltage(power);

        angPID.prevError = angPID.error;
        previousTime = currentTime;

        pros::delay(10);
    }

    rightChassis.brake();
    leftChassis.brake();
    pros::delay(100); // Allow some time for motors to stop completely
}



/**
 * Calculates the linear error between a target value and the current position.
 *
 * @param target The target value to reach.
 * @param leftTicks The number of ticks on the left side of the robot.
 * @param rightTicks The number of ticks on the right side of the robot.
 * @return The linear error between the target value and the current position.
 */
double getLinearError(double target, double leftTicks, double rightTicks) {
    //Get the average between the two sides
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
    // Wrap error into the range [-180, 180]
    while (error > 180)  error -= 360;
    while (error < -180) error += 360;
    return error;
}


// Converts degrees to radians
/**
 * @brief Converts degrees to radians.
 * 
 * @param deg The value in degrees to be converted.
 * @return The value in radians.
 */
double degToRad(double deg) {
    return deg * (M_PI / 180);
}

// Converts radians to degrees
/**
 * @brief Converts an angle from radians to degrees.
 * 
 * @param rad The angle in radians.
 * @return The angle in degrees.
 */
double radToDeg(double rad) {
    return rad * (180 / M_PI);
}

/**
 * @brief Updates the odometry based on the number of ticks on the left and right wheels.
 * 
 * This function calculates the distances moved by the left and right wheels based on the number of ticks,
 * and then calculates the average distance traveled. It updates the global position by adding the
 * calculated distance multiplied by the cosine and sine of the current heading. Finally, it prints
 * the global heading in radians on the screen.
 * 
 * @param leftTicks The number of ticks on the left wheel.
 * @param rightTicks The number of ticks on the right wheel.
 */
void updateOdom(double leftTicks, double rightTicks) {
    // Use delta ticks since last update to integrate motion
    static double lastLeftTicks = 0.0;
    static double lastRightTicks = 0.0;

    double deltaLeftTicks = leftTicks - lastLeftTicks;
    double deltaRightTicks = rightTicks - lastRightTicks;

    lastLeftTicks = leftTicks;
    lastRightTicks = rightTicks;

    // Convert to distance
    double distLeft = deltaLeftTicks * distOneTick;
    double distRight = deltaRightTicks * distOneTick;
    double averageDist = (distLeft + distRight) / 2.0;    // Average distance traveled this cycle

    // Heading in degrees from IMU
    globalHeading = imuSensor.get_heading();

    // Update global position using current heading (convert deg->rad once)
    double headingRad = degToRad(globalHeading);
    globalPos[0] += averageDist * cos(headingRad);
    globalPos[1] += averageDist * sin(headingRad);

    pros::screen::print(pros::E_TEXT_MEDIUM, 8, "Global heading (deg): %f", globalHeading);
}

/*
Pseudo code for updateOdom:

updateOdom(leftTicks, rightTicks){
    distLeft = leftTicks * distOneTick
    distRight = rightTicks * distOneTick

    averageDist = (distLeft + distRight) / 2 

    //gives us our change in heading with respect to the wheelbase
    deltaTheta = (distRight - distLeft) / wheelBase

    deltaTheta = int(deltaTheta) % 360 //makes sure we are within the 360 degree range

    globalHeading += deltaTheta //updating the heading

    //updating the positioning in x,y cordinate plane
    globalPos[x] = globalPos[x] + averageDist * cos(globalHeading)
    globalPos[y] = globalPos[y] + averageDist * sin(globalHeading)

    print("Updated global heading, global heading: ", globalHeading)
}
*/