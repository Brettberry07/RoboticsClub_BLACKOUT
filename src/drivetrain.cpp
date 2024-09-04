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
    leftChassis.move(master.get_analog(ANALOG_LEFT_Y));
    rightChassis.move(master.get_analog(ANALOG_RIGHT_Y));
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