#include "globals.hpp"
#include "pneumatics.hpp"

//If a button is pressed, we toggle the pneumatics state,
//this allows us to go from pneumatics on the drivetrain,
//to pneumatics on the intake with a press of a button
void switchState(bool state){
    /*
    Low state is on drivetrain
    High state is on intake
    */
    if(master.get_digital(DIGITAL_L1)){
        state = LOW ? HIGH : LOW;     //if low, equals high, else equals low
        pneumatics.set_value(state);
        if(state){
        pros::lcd::set_text(2,"pneumatics on");
        }
        pros::lcd::set_text(2,"pneumatics off");
    }
}

