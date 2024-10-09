#include "globals.hpp"

//This allows me to access different dirving methods quickly
//We plan to do tank drive so I default to tank drive for redundancy
void driveTrain(char driveScheme, bool isCurved, bool pneumaticsState){
    checkCurveInput();      //Cheking to see if we a driving curve for accuracy, or linear for speed
    switch(driveScheme){
        case 't':
        if(isCurved){
            tankDriveCubic(pneumaticsState); 
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
    //Get percentage of controller input, cube value, then multiply back to whole number in range of 127
    return ((controllerInput/127)*(controllerInput/127)*(controllerInput/127))/127;
}

void checkCurveInput(){
    if(master.get_digital(DIGITAL_Y)){
        isCurved = true;
        if (isCurved){
            pros::lcd::set_text(1,"Cubic curve");
        }
        else{
            pros::lcd::set_text(1,"linear drive");
        }
        
    }
}