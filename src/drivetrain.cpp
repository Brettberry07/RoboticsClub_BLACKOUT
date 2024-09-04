#include "globals.hpp"

void driveTrain(char driveScheme){
    if(driveScheme == 't'){
        tankDrive();
    }
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

//TODO: Add sensitivity curves (accelaration curves)
void tankDrive(){
    leftChassis.move(cubicCurve(master.get_analog(ANALOG_LEFT_Y)));
    rightChassis.move(cubicCurve(master.get_analog(ANALOG_RIGHT_Y)));
}

void arcadeDriveTwo(){
    int power = master.get_analog(ANALOG_LEFT_Y);
    int turn = master.get_analog(ANALOG_RIGHT_X);
    leftChassis.move(power+turn);
    rightChassis.move(power-turn);
}

void arcadeDriveOne(){
    int power = master.get_analog(ANALOG_LEFT_Y);
    int turn = master.get_analog(ANALOG_LEFT_X);
    leftChassis.move(power+turn);
    rightChassis.move(power-turn);
}

int cubicCurve(int controllerInput){
    //Get percentage of controller input cube value, then multiply back to 127
    return ((controllerInput/127)*(controllerInput/127)*(controllerInput/127))/127;
}

int controllerCurve(int controllerInput, int power){
    int total = 1;
    for(int i=0; i<power; i++){
        total *= (controllerInput/127);
    }
    return total/127;
}