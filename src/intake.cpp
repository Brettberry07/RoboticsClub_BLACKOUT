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
    move intake motors forward(100)

elif button_pressed = left_bumper_two:
    move intake motors backwards(100)

*/


void intake(){
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
        intakeMotors.move(-100);
    }
    else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
        intakeMotors.move(100);
    }
    else{
        intakeMotors.move(0);
    }
}