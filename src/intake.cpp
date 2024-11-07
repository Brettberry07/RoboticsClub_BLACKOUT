#include "globals.hpp"

/*
DESCRIPTION:
    Move the motors between a range of -127 to 127.acos
    We move by 100 and -100 to leep the intake form moving to fast
    If the first button is pressed, that mean we move the motors
    to intake the donut, if the other button is pressed, 
    we move the motor to output the intake incase we need to.

CODE:
    if button_pressed = left_bumper_one:
        move intake motors backwards(100)

    elif button_pressed = left_bumper_two:
        move intake motors forwards(100)

    else:
        stop moving motors

*/

//get controller press, then move intake accordingly
void intake(){
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
        intakeMotors.move(-60);
    }
    else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
        intakeMotors.move(60);
    }
    else{
        intakeMotors.move(0);
    }
}

// ---------------------------------------Used for autonomous---------------------------------------------------- //

/*
Description:
    used for only during autonomous, moves with time because 
    it's easier to measure then exact length. We also don't have
    sensors to detect if the donut is at the top. 
    Tested and found a good time.

Pseudocode:
void autonIntake():
    move intake motors
    wait certain time
    stop moving motors
*/

//be able to move the intake for a certain 
//amount of time for auton period.
void autonIntake(){
    intakeMotors.move(60);
    pros::delay(1000);
    intakeMotors.move(0);
    pros::delay(100);
}
