#include "globals.hpp"
#include <algorithm>

/*
TODO:
    create a linear and anglular pid.
*/

/*
Sudo code for now:


linPID(){
    proportion = currentPos - desiredPos
    integral += error         //This isn't used to eoften so might not use it
    derivative = error-prevError

    kP = constant for proportion
    kI = constant ofr integral          //These are how I tune the PID
    kD = constant for derivative



    Lets just say for this I know how far I want to move for
    simplicity, I'm assuming I would solve this through some
    sort of odometry thing

    while(error>someNumber){        //we can't reach perfection so choose a value close to it
        proportion = currentPos - desiredPos               //This is how far we are
        integral += error                                  //This isn't used to eoften so might not use it, but all errors
        derivative = error - prevError                     //How fast were approaching goal basically

        power = (proportion*kP)+(integral*kI)+(derivative*kD)
        RightMotor.move(power)      //idk how I want to move this, prly with voltage honestly
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

//For ticks per revolution
//Blue: 1800 
//Green: 900
//Red: 300



// Jhon and Joe both said this was bad so make into struct
// Cameron here, I made it into a struct because I'm awesome 

// struct PidConstants{
//     const double linear_kP = 0;
//     const double linear_kI = 0;    //constants forn tuning linear PID
//     const double linear_kD = 0;

//     const double angular_kP = 0;
//     const double angular_kI = 0;    //constants forn tuning angular PID
//     const double angular_kD = 0;
// };

// These constants bassically weigh how our PID system reacts to the current error
// P reacts to current overall error, main driving force so to speak 
// I controls the compensation for the total accumulated error (the error the P can't solve)
// D removed for now, causes oscilation in feedback (Is supposed to help with overshooting our error, isn't implemented well)
PIDConstants linPID = {150, 5, 0};
PIDConstants angPID = {20, 10, 0};

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

    while(true) {
        leftTicks = leftChassis.get_position();
        rightTicks = rightChassis.get_position();

        
        linPID.error = getLinearError(target, leftTicks, rightTicks);
        pros::screen::print(TEXT_MEDIUM, 1, "Error: %f", linPID.error);

        linPID.derivative = (linPID.prevError - linPID.error) / 0.001;
        pros::screen::print(TEXT_MEDIUM, 2, "Derivative: %f", linPID.derivative);

        linPID.integral += (linPID.error * 0.001);
        pros::screen::print(TEXT_MEDIUM, 3, "Integral: %f", linPID.integral);

        //Clamp sets the max and min value of the var (in this case integral)
        linPID.integral = std::clamp(linPID.integral, linPID.low, linPID.high);

        power = (linPID.kP * linPID.error) + (linPID.kI * linPID.integral) + (linPID.kD * linPID.derivative);
        pros::screen::print(TEXT_MEDIUM, 4, "Power: %d", power);


        if(time>linPID.timeOut) {
            pros::screen::print(TEXT_MEDIUM, 5, "Time Out, Time reached: %f", linPID.timeOut);
            break;
        } 

        if(abs(linPID.error) < 1) {
            pros::screen::print(TEXT_MEDIUM, 5, "Min range met. Range: %f", linPID.error);
            break;
        }

        driveTrainMotors.move_voltage(power);
        linPID.prevError = linPID.error; // Added as I couldn't find any lines where we change the value of prevError
        pros::delay(10);
        time+=10;
        pros::screen::print(TEXT_MEDIUM, 5, "Time: %f", time);

    }

    rightChassis.brake();
    leftChassis.brake();
}

// Cameron here, made a struct for this, haven't converted variables.
void AngularPid(double target) {
    int32_t power = 0;
    uint16_t time = 0;

    double leftTicks = 0;
    double rightTicks = 0;

    //resseting position before pid loop starts
    rightChassis.tare_position();
    leftChassis.tare_position();

    while(true) {
        leftTicks = leftChassis.get_position();
        rightTicks = rightChassis.get_position();

        angPID.error = getAngularError(target, leftTicks, rightTicks);
        pros::screen::print(TEXT_MEDIUM, 1, "Error: %f", angPID.error);

        angPID.derivative = (angPID.prevError - angPID.error) / 0.001;
        pros::screen::print(TEXT_MEDIUM, 2, "Derivative: %f", linPID.derivative);

        angPID.integral += (angPID.error * 0.001);
        pros::screen::print(TEXT_MEDIUM, 3, "Integral: %f", linPID.integral);

        //If the Integral goes beyond the maximum output of the system,
        //Then the intergal is going to windup, so we just reset the intergal
        linPID.integral = std::clamp(angPID.integral, angPID.low, angPID.high);

        power =(angPID.kP * angPID.error) + (angPID.kI * angPID.integral) + (angPID.kD * angPID.derivative);
        pros::screen::print(TEXT_MEDIUM, 4, "Power: %d", power);

        if(time>linPID.timeOut) {
            pros::screen::print(TEXT_MEDIUM, 5, "Time Out, Time reached: %f", angPID.timeOut);
            break;
        } 

        if(abs(linPID.error) < 1) {
            pros::screen::print(TEXT_MEDIUM, 5, "Min range met. Range: %f", angPID.error);
            break;
        }

        rightChassis.move_voltage(power);
        leftChassis.move_voltage(-power);

        angPID.prevError = angPID.error;
        pros::delay(10);
        time+=10;
    }

    rightChassis.brake();
    leftChassis.brake();
}

//TODO: Look into making the PID systems seperate so I can have the most accurate possible
double getLinearError(double target, double leftTicks, double rightTicks) {
    //Get the average between the two sides
    updateOdom(leftTicks, rightTicks);
    double temp = ((distOneTick * rightTicks) + (distOneTick * leftTicks))/2;
    pros::screen::print(TEXT_MEDIUM, 5, "pos: %f", temp);
    return target - temp;
    
}

double getAngularError(double target, double leftTicks, double rightTicks) {
    updateOdom(leftTicks, rightTicks);
    return target - globalHeading;
}

//allows for the tracking of the headin and the x,y cordinates
void updateOdom(double leftTicks, double rightTicks){
    //calculating how far each side has moved
    double distLeft = leftTicks*distOneTick;
    double distRight = rightTicks*distOneTick;

    //getting average disatnce travelled
    double averageDist = (distLeft + distRight) / 2;

    //getting the change in the heading
    double deltaTheta = (distRight - distLeft) / wheelBase;

    //updatig the heading
    globalHeading += deltaTheta;

    //updating the positioning
    globalPos[0] = globalPos[0] + averageDist * cos(globalHeading);
    globalPos[1] = globalPos[1] + averageDist * sin(globalHeading);
}