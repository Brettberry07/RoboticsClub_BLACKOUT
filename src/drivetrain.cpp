#include "globals.hpp"
/*
DESCRIPTION:
We need to take into account that the pneumatics state will depend
what motors we move. We need different drive schemes so we can 
test what drive is the best.(We decided tank drive so that's the one
I built out) I created a way to make a cubic curve on the driving input
so that we can have accurate vs fast driving. So once we take into account
that, we can decide if we need a cubic drive, or normal drive. Then inside those
functions we can deside to move drive+pneumatic motors, or only drive motors.

PSEUDOCODE:
void drivetrain(drivescheme){
    if tank drive {do tank control scheme; tank_drive()}
    else if split drive {do split drive control scheme; split_drive()}
    else if arcade drive {do arcade drive scheme; arcade_drive()}
    else {for redundancy default to tank, tank_drive()}
}

void tank_drive(){
    get the y axis of both sticks;
    if cubed:
        if pneumatics on:
            apply power to the 6 motors with cubic curve
        else:
            apply power to only 4 motors with cubic curve
    else:
        if pneumatics on:
            apply power to the 6 motors with linear input
        else:
            apply power to only 4 motors with linear inout
}

void cube_curve(controller_axis){
    return controller_axis^3 / 127^2
}

void check_curve_input(){
    if a_button pressed:
        if already curved:
            now linear drive
        else:
            now curve drive
}

void split_drive() {get y axis of left stick, x axis of right stick; apply these values to motors}

void arcade_drive() {get y and x axis of left left stick; apply values to motors}

*/

//This allows me to access different dirving methods quickly
//We plan to do tank drive so I default to tank drive for redundancy
void driveTrain(char driveScheme, bool isCurved, bool pneumaticsState){
    checkCurveInput();      //Cheking to see if we a driving curve for accuracy, or linear for speed
    switch(driveScheme){
        case 't':
        if(isCurved){
            tankDrive(pneumaticsState); 
        }
        else{
            tankDrive(pneumaticsState);
        }
        case 's':
        arcadeDriveTwo();
        case 'a':
        arcadeDriveOne();
        default:
        tankDrive(pneumaticsState);
    }
}

//tank drive...
void tankDrive(bool pneumaticsState){
    /*
    If the pneumatics state is high that means we are using a 6 motor drive 
    so we need to move the 6 motors instead of only moving the normal 4
    */
    if (pneumaticsState == HIGH){
        pneumaticsLeftChassis.move(master.get_analog(ANALOG_LEFT_Y));
        pneumaticsRightChassis.move(master.get_analog(ANALOG_RIGHT_Y));
    }
    else{
    leftChassis.move(master.get_analog(ANALOG_LEFT_Y));
    rightChassis.move(master.get_analog(ANALOG_RIGHT_Y));
    }
}

void tankDriveCubic(bool pneumaticsState){
    /*
    This is for if we are using prevision driving, we do this by applying a cubic curve
    and slowing down the acceleration, if not were using normal linesr acceleration
    */
    if (pneumaticsState == HIGH){
        pneumaticsLeftChassis.move(cubicCurve(master.get_analog(ANALOG_LEFT_Y)));
        pneumaticsRightChassis.move(cubicCurve(master.get_analog(ANALOG_LEFT_Y)));

    }
    else{
    leftChassis.move(cubicCurve(master.get_analog(ANALOG_LEFT_Y)));
    rightChassis.move(cubicCurve(master.get_analog(ANALOG_RIGHT_Y)));
    }
}

//This is split drive
void arcadeDriveTwo(){
    int power = master.get_analog(ANALOG_LEFT_Y);
    int turn = master.get_analog(ANALOG_RIGHT_X);
    leftChassis.move(power+turn);
    rightChassis.move(power-turn);
}

//One joystick arcade
void arcadeDriveOne(){
    int power = master.get_analog(ANALOG_LEFT_Y);
    int turn = master.get_analog(ANALOG_LEFT_X);
    leftChassis.move(power+turn);
    rightChassis.move(power-turn);
}

//Going to use this for sensitivity curve
int cubicCurve(int controllerInput){
    //Get percentage of controller input, cube that value, then back to whole number in range of 127
    return ((controllerInput*controllerInput*controllerInput)/(127*127));

}

void checkCurveInput(){
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
        if(isCurved){
            isCurved = false;
        }
        else{
            isCurved = true;
        }
    }
}


// ---------------------------------------Used for autonomous---------------------------------------------------- //

//for moving forward or backwards
//takes an input of inches
//-velocity means backwards, + means forwards
void driveTrainMove(double dist, int velocity){
    driveTrainMotors.tare_position();

    int ticks = dist / distOneTick; //gets the ticks needed to reach the distance

    driveTrainMotors.move_relative(ticks, velocity);

    while (!((driveTrainMotors.get_position() < ticks + 5) && (driveTrainMotors.get_position() > ticks - 5))) {
    // Continue running until motors within +-5 ticks of deisred position
    // we have to wait because if we don't we will move to the next function
        pros::delay(2);
    }
    
    driveTrainMotors.brake();
    pros::delay(150);
}

//turning left and right
//takes an input of degrees to turn
//-velocity means counter-clockwise, + means clock-wise
void driveTrainTurn(double theta, int velocity){
    rightChassis.tare_position();
    leftChassis.tare_position();

    //how many ticks we need to turn to the angle.
    //this caluclated by theta (desired angle) divided by 360
    //we then multiply this by pi, and the wheel base (distance between left and right wheels),
    //we the divide this by how far we travel by one tick per motor.

    int ticks = ((theta / 360) * M_PI * wheelBase) / (distOneTick);

    rightChassis.move_relative(-ticks, velocity);
    leftChassis.move_relative(ticks, velocity);

    while (!((rightChassis.get_position() > ticks - 5) && (rightChassis.get_position() < ticks + 5) ||
            (leftChassis.get_position() > ticks - 5) && (leftChassis.get_position() < ticks + 5))) {
        pros::delay(2);
    }

    rightChassis.brake();
    leftChassis.brake();
    pros::delay(150);
}
