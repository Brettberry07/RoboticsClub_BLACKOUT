#include "globals.hpp"



//This allows me to access different dirving methods quickly
//We plan to do tank drive so I default to tank drive for redundancy
void driveTrain(char driveScheme, bool isCurved){
    checkCurveInput();
    if(driveScheme == 't'){
        if(isCurved){
            tankDriveCubic();
        }
        else{
            tankDrive();
        }
    }
//Not worrying about the other other ones until we figure out what we want to do
    else if(driveScheme == 's'){
        arcadeDriveTwo();
    }
    else if(driveScheme == 'a'){
        arcadeDriveOne();
    }
    else{
        tankDrive();
    }
}

//tank drive...
void tankDrive(){
    leftChassis.move(master.get_analog(ANALOG_LEFT_Y));
    rightChassis.move(master.get_analog(ANALOG_RIGHT_Y));
}

void tankDriveCubic(){
    leftChassis.move(cubicCurve(master.get_analog(ANALOG_LEFT_Y)));
    rightChassis.move(cubicCurve(master.get_analog(ANALOG_RIGHT_Y)));
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

//Going to use this for sensitivity curve, probably going to be able to switch between this and linear
int cubicCurve(int controllerInput){
    //Get percentage of controller input, cube value, then multiply back to whole number in range of 127
    return ((controllerInput/127)*(controllerInput/127)*(controllerInput/127))/127;
}

void checkCurveInput(){
    if(master.get_digital(DIGITAL_Y)){
        isCurved = true;
    }
}