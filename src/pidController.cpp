#include "globals.hpp"
#include <cmath>

/*
TODO:
    create a linear and anglular pid.
*/

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
PIDConstants linPID = {1, 1, 1};
PIDConstants angPID = {1, 1, 1}; 

// Cameron here, should make this adaptive based on distance or angular target magnetude 
// Harder to reach goals should have a higher timeout
// const int timeOut = 50000;

void linearPID(double target) {
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

    while(true) {
        leftTicks = leftChassis.get_position();
        rightTicks = rightChassis.get_position();

        
        linPID.error = getLinearError(target, leftTicks, rightTicks);
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Error: %f", linPID.error);

        // linPID.derivative = (linPID.error - linPID.prevError) / 0.01;
        linPID.derivative = (linPID.error - linPID.prevError) / 2;
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Derivative: %f", linPID.derivative);

        // linPID.integral += (linPID.error * 0.01);
        linPID.integral += (linPID.error);
        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Integral: %f", linPID.integral);

        //Clamp sets the max and min value of the var (in this case integral)
        linPID.integral = std::clamp(linPID.integral, linPID.low, linPID.high);

        power = (linPID.kP * linPID.error) + (linPID.kI * linPID.integral) + (linPID.kD * linPID.derivative);
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Power: %d", power);


        if(time > linPID.timeOut) {
            pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Time Out, Time reached: %f", linPID.timeOut);
            break;
        } 

        if(abs(linPID.error) < 1) {
            pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Min range met. Range: %f", linPID.error);
            break;
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

// Basically the same as linearPID but for angular movement
void angularPID(double target) {
    int32_t power = 0;
    // uint16_t time = 0;

    // Time variables for caluclating delta time
    uint16_t previousTime = 0;
    double currentTime = pros::millis();

    double leftTicks = 0;
    double rightTicks = 0;

    //resseting position before pid loop starts
    rightChassis.tare_position();
    leftChassis.tare_position();
    
    //Ensure nothing carries over from last loop
    angPID.error = 0;
    angPID.prevError = 0;
    angPID.integral = 0;

    while(true) {
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Starting!");
        leftTicks = leftChassis.get_position();
        rightTicks = rightChassis.get_position();
        // double currentTime = pros::millis();

        angPID.error = getAngularError(target, leftTicks, rightTicks);
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Error: %f", angPID.error);
        
        // double dt = (currentTime - previousTime) / 1000.0; // Convert ms to seconds
        // if(dt >= 0.1) {
        //     pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Calculation irregularities detected: %f", dt);
        //     break;
        // }

        angPID.derivative = (angPID.error - angPID.prevError); // Calculate derivative
        previousTime = currentTime; // Update previous time

        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Derivative: %f", angPID.derivative);

        angPID.integral += (angPID.error);
        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Integral: %f", angPID.integral);

        angPID.integral = std::clamp(angPID.integral, angPID.low, angPID.high);

        // If the Integral goes beyond the maximum output of the system,
        // Then the intergal is going to windup, so we just reset the intergal

        // Compute control signal
        power = (angPID.kP * angPID.error) + 
                (angPID.kI * angPID.integral) + 
                (angPID.kD * angPID.derivative);
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Power: %d", power);

        // WE WERE USING LINPID, NOT ANGPID HERE, NO WONDER IT WASN'T WORKING, I'M DUMB, AGHHHHHHHHHHHH
        // Plus the error wasn't going to work because it was trying to compare radians to degrees, so fixed that as well
        // if (currentTime > angPID.timeOut) {
        //     pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Time Out, Time reached: %f", angPID.timeOut);
        //     break;
        // }

        // if(abs(angPID.error - angPID.prevError) < 0.1) {
        //     pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Min range met. Range: %f", angPID.error);
        //     break;
        // }

        // Adaptive error threshold with a min range of 1% of the target heading
        // double errorThreshold = std::max(1.0, target * 0.01); // 1% of the target or a minimum of 1
        // if (abs(angPID.error) < errorThreshold) {
        //     break;
        // }


        rightChassis.move_voltage(power);
        leftChassis.move_voltage(-power);

        angPID.prevError = angPID.error;
        pros::delay(10);
        // time+=10;
    }

    rightChassis.brake();
    leftChassis.brake();
}


//TODO: Look into making the PID systems seperate so I can have the most accurate possible
double getLinearError(double target, double leftTicks, double rightTicks) {
    //Get the average between the two sides
    updateOdom(leftTicks, rightTicks);
    double temp = ((distOneTick * rightTicks) + (distOneTick * leftTicks))/2;
    pros::screen::print(pros::E_TEXT_MEDIUM, 5, "pos: %f", temp);
    return target - temp;

}

double getAngularError(double target, double leftTicks, double rightTicks) {
    updateOdom(leftTicks, rightTicks);
    // Compute error and wrap within [-π, π] (Radians)
    double error = target - globalHeading;
    return error;
}

// Converts degrees to radians
double degToRad(double deg) {
    return deg * (M_PI / 180);
}

// Converts radians to degrees
double radToDeg(double rad) {
    return rad * (180 / M_PI);
}

void updateOdom(double leftTicks, double rightTicks) {
    // Calculate distances moved
    double distLeft = leftTicks * distOneTick;
    double distRight = rightTicks * distOneTick;

    double averageDist = (distLeft + distRight) / 2;    // Average distance traveled

    globalHeading = imuSensor.get_heading(); // Get the current heading from the IMU (degrees), Then convert to radians

    // Update global position (using radians)
    globalPos[0] += averageDist * cos(globalHeading);
    globalPos[1] += averageDist * sin(globalHeading);

    pros::screen::print(pros::E_TEXT_MEDIUM, 8, "Global heading (rad): %f", globalHeading);
}


// Another Old ODom Function
// // new updateOdom function
// // allows for the tracking of the heading and the x,y coordinates
// void updateOdom(double leftTicks, double rightTicks) {
//     // Calculate distances moved
//     double distLeft = leftTicks * distOneTick;
//     double distRight = rightTicks * distOneTick;

//     double averageDist = (distLeft + distRight) / 2;    // Average distance traveled
//     double deltaTheta = (distRight - distLeft) / wheelBase; // Change in heading (radians)

//     // Update global heading (keep in radians)
//     globalHeading += deltaTheta;

//     /* WARNING TECHNICAL SPEACH:
//     atan2 essentially gives us the angle in radians that corresponds to 
//     the coordinates (sin(globalHeading), cos(globalHeading)) in a polar coordinate system. */
//     globalHeading = atan2(sin(globalHeading), cos(globalHeading)); // Normalize globalHeading to [-π, π] (Radians) which is [-180, 180] (degrees)

//     // Update global position (using radians)
//     globalPos[0] += averageDist * cos(globalHeading);
//     globalPos[1] += averageDist * sin(globalHeading);

//     pros::screen::print(pros::E_TEXT_MEDIUM, 6, "Global heading (rad): %f", globalHeading);
// }

/* Old updateOdom function
void updateOdom(double leftTicks, double rightTicks) {
    // Calculate how far each side has moved
    double distLeft = leftTicks * distOneTick;
    double distRight = rightTicks * distOneTick;

    // Calculate average distance traveled and change in heading
    double averageDist = (distLeft + distRight) / 2;
    double deltaTheta = radToDeg((distRight - distLeft) / wheelBase);

    // Update global heading
    globalHeading += deltaTheta;

    // Wrap global heading to [0, 360)
    globalHeading = std::clamp(globalHeading, 0.0, 360.0);

    // Update position
    double headingRad = degToRad(globalHeading);
    globalPos[0] += averageDist * cos(headingRad);
    globalPos[1] += averageDist * sin(headingRad);

    // Debug output
    pros::screen::print(pros::E_TEXT_MEDIUM, 6, "Updated global heading, global heading: %f", globalHeading);
} */

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