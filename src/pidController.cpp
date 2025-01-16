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
PIDConstants linPID = {4, 1, 1};
PIDConstants angPID = {2, 1, 1}; 

// Cameron here, should make this adaptive based on distance or angular target magnetude 
// Harder to reach goals should have a higher timeout
// const int timeOut = 50000;

void linearPID(double target) {

    bool isNeg = false;

    if(target < 0) {
        target = abs(target);
        isNeg = true;
    }

    int32_t power = 0;
    uint16_t time = 0;

    double leftTicks = 0;
    double rightTicks = 0;

    //resseting position before pid loop starts
    driveTrainMotors.tare_position();

    //Ensure nothing carries over from last loop
    linPID.error = 0;
    linPID.prevError = 0;
    linPID.integral = 0;

    double dt = 0;
    uint32_t currentTime = 0;
    uint32_t previousTime = pros::millis();

    while(true) {
        leftTicks = leftChassis.get_position();
        rightTicks = rightChassis.get_position();

        linPID.error = isNeg ? getLinearError(target, leftTicks, rightTicks) : getLinearError(-target, leftTicks, rightTicks);
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Error: %f", linPID.error);

        currentTime = pros::millis();
        double dt = (currentTime - time) / 1000; // Convert ms to seconds

        linPID.derivative = isNeg ?  (linPID.error + linPID.prevError): (linPID.error - linPID.prevError);
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Derivative: %f", linPID.derivative);

        previousTime = currentTime; // Update previous time

        linPID.integral += (linPID.error);
        //Clamp sets the max and min value of the var (in this case integral)
        linPID.integral = std::clamp(linPID.integral, linPID.low, linPID.high);
        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Integral: %f", linPID.integral);

        previousTime = currentTime; // Update previous time

        power = (linPID.kP * linPID.error) + 
            (linPID.kI * linPID.integral) + 
            (linPID.kD * linPID.derivative);
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Power: %d", power);


        if(time > linPID.timeOut) {
            pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Time Out, Time reached: %f", linPID.timeOut);
            break;
        } 

        if(abs(linPID.error) < 1) {
            pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Min range met. Range: %f", linPID.error);
            break;
        }

        if(isNeg) {
            power = -power;
        }
        driveTrainMotors.move_voltage(power);
        linPID.prevError = linPID.error; // Added as I couldn't find any lines where we change the value of prevError
        pros::delay(10);
        time+=10;
        pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Time: %f", time);

    }

    rightChassis.brake();
    leftChassis.brake();
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
    // uint16_t time = 0;

    double leftTicks = 0;
    double rightTicks = 0;

    //resseting position before pid loop starts
    rightChassis.tare_position();
    leftChassis.tare_position();
    
    //Ensure nothing carries over from last loop
    angPID.error = 0;
    angPID.prevError = 0;
    angPID.integral = 0;

    double dt = 0;
    uint32_t currentTime = 0;
    uint32_t previousTime = pros::millis();

    while(true) {
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Starting!");
        leftTicks = leftChassis.get_position();
        rightTicks = rightChassis.get_position();
        angPID.error = getAngularError(target, leftTicks, rightTicks);
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Error: %f", angPID.error);
        
        currentTime = pros::millis();
        dt = (currentTime - previousTime) / 1000.0; // Convert ms to seconds

        angPID.derivative = (angPID.error - angPID.prevError); // Calculate derivative using change in time
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Derivative: %f", angPID.derivative);

        previousTime = currentTime; // Update previous time
        
        currentTime = pros::millis();
        dt = (currentTime - previousTime) / 1000.0; // Convert ms to seconds

        angPID.integral += (angPID.error);
        angPID.integral = std::clamp(angPID.integral, angPID.low, angPID.high);
        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Integral: %f", angPID.integral);

        // If the Integral goes beyond the maximum output of the system,
        // Then the intergal is going to windup, so we just reset the intergal

        // Compute control signal
        power = (angPID.kP * angPID.error) + 
                (angPID.kI * angPID.integral) + 
                (angPID.kD * angPID.derivative);
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Power: %d", power);

        // WE WERE USING LINPID, NOT ANGPID HERE, NO WONDER IT WASN'T WORKING, I'M DUMB, AGHHHHHHHHHHHH
        // Plus the error wasn't going to work because it was trying to compare radians to degrees, so fixed that as well
        if (currentTime > angPID.timeOut) {
            pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Time Out, Time reached: %f", angPID.timeOut);
            break;
        }

        if(abs(angPID.error) < 1) {
            pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Min range met. Range: %f", angPID.error);
            break;
        }

        // Adaptive error threshold with a min range of 1% of the target heading
        // double errorThreshold = std::max(1.0, target * 0.01); // 1% of the target or a minimum of 1
        // if (abs(angPID.error) < errorThreshold) {
        //     break;
        // }

        rightChassis.move_voltage(-power);  //THIS ONE NEGATIVE
        leftChassis.move_voltage(power);

        angPID.prevError = angPID.error;
        pros::delay(10);
    }
    rightChassis.brake();
    leftChassis.brake();
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

/**
 * Calculates the angular error between the target angle and the current global heading.
 *
 * @param target The target angle in radians.
 * @param leftTicks The number of left wheel encoder ticks.
 * @param rightTicks The number of right wheel encoder ticks.
 * @return The angular error in radians.
 */
double getAngularError(double target, double leftTicks, double rightTicks) {
    updateOdom(leftTicks, rightTicks);
    // Compute error and wrap within [-π, π] (Radians)
    double error = target - globalHeading;
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
    // Calculate distances moved
    double distLeft = leftTicks * distOneTick;
    double distRight = rightTicks * distOneTick;

    double averageDist = (distLeft + distRight) / 2;    // Average distance traveled

    // globalHeading = fmod(imuSensor.get_yaw() + 360, 360); // Get the current heading from the IMU (degrees), Then convert to radians
    globalHeading = imuSensor.get_heading(); // Get the current heading from the IMU (degrees), Then convert to radians

    // Update global position (using radians)
    globalPos[0] += averageDist * cos(degToRad(globalHeading));
    globalPos[1] += averageDist * sin(degToRad(globalHeading));

    pros::screen::print(pros::E_TEXT_MEDIUM, 8, "Global heading (rad): %f", globalHeading);
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