#include "globals.hpp"

/*
TODO:
    create a linear and anglular pid.
*/

/*
Sudo code for now:


LinearPid(){
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
const uint8_t radius = 3.25;   //Radius of the wheel
const int distPerTick = ((2*radius)*gearRatio*M_1_PI)/1800; //This gives us are distance in inches



//TODO: Jhon and Joe both said this was bad so make into struct

// struct PidConstants{
//     const double linear_kP = 0;
//     const double linear_kI = 0;    //constants forn tuning linear PID
//     const double linear_kD = 0;

//     const double angular_kP = 0;
//     const double angular_kI = 0;    //constants forn tuning angular PID
//     const double angular_kD = 0;
// };

const double linear_kP = 0;
const double linear_kI = 0;    //constants forn tuning linear PID
const double linear_kD = 0;

const double angular_kP = 0;
const double angular_kI = 0;    //constants forn tuning angular PID
const double angular_kD = 0;

const int timeOut = 50000;       //idk a good timeout time yet

void linearPID(double target){
    double error = 0;
    double prevError = 0;
    double integral = 0;
    double derivative = 0;

    int32_t power = 0;
    uint16_t time = 0;

    while(true){
        error = getLinearError(target);
        derivative = prevError-error;
        integral += error;

        //If the Integral goes beyond the maximum output of the system,
        //Then the intergal is going to windup, so we just reset the intergal
        if(fabs(integral*linear_kI) > fabs(12000)){
            integral = 0;
        }

        power = (linear_kP*error)+(linear_kI*integral)+(linear_kD*derivative);

        if(time>timeOut){
            pros::lcd::set_text(2, "Time Out");
            break;
        }
        else if(abs(error) < 1){
            pros::lcd::set_text(2, "Success");
            break;
        }

        rightChassis.move_voltage(power);
        leftChassis.move_voltage(power);

        pros::delay(10);
        time+=10;

    }
    rightChassis.brake();
    leftChassis.brake();
}

void AngularPid(double target){
    double error = 0;
    double prevError = 0;
    double integral = 0;
    double derivative = 0;

    int32_t power = 0;
    uint16_t time = 0;

    while(true){
        error = getAngularError(target);
        derivative = prevError-error;
        integral += error;
        // derivative = (prevError-error)/0.001;    Both these are very small time intervals and may mess with the
        // integral += error * 0.001;               inegral and derivative values, I'll look more into this

        //If the Integral goes beyond the maximum output of the system,
        //Then the intergal is going to windup, so we just reset the intergal
        if(fabs(integral*angular_kI) > fabs(120000)){
            integral = 0;
        }

        power = angular_kP*error+angular_kI*integral+angular_kD*derivative;

        if(time>timeOut){
            pros::lcd::set_text(2, "Time Out");
            break;
        }
        else if(abs(error) < 1){
            break;
        }

        rightChassis.move_voltage(power);
        leftChassis.move_voltage(-power);

        pros::delay(10);
        time+=10;

    }
    rightChassis.brake();
    leftChassis.brake();
}

//TODO: Look into making the PID systems seperate so I can have the most accurate possible
double getLinearError(double target){
        //Get the average between the two sides
    return target - ((rightChassis.get_position() * distPerTick)*(leftChassis.get_position() * distPerTick))/2;
}

double getAngularError(double target){
        //We have to normalize angle because yaw is between (-180,180), and it's easier to wrtie a (0,360)
    return target - normalizeAngle(imuSensor.get_yaw());
}

/*
(-180, 180)
90 -> -270
-90 -> 270
180 -> 180
-180 -> 180
179 -> 
*/
double normalizeAngle(double angle){
    /*(-180, 180)*/
    if(angle > 0){
        return angle - 360;
    }
    else if(angle < 0){
        return angle + 360;
    }
    else{
        return angle;
    }
}