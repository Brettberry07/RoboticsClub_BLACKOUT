#include "globals.hpp"
#include "pneumatics.hpp"

/*
pseudocode:

bool switch_state(bool state){
    if low(off): now high(on or extended)
    if high: now low

    return new state
}

*/

//If a button is pressed, we toggle the pneumatics state,
//this allows us to go from pneumatics on the drivetrain,
//to pneumatics on the intake with a press of a button
bool switchState(bool state, pros::controller_digital_e_t button, pros::adi::Port pin){
    /*
    Low state is on drivetrain
    High state is on intake
    */
    if(master.get_digital(button)){
        // state = LOW ? HIGH : LOW;     //if low, equals high, else equals low
        if(state == HIGH){
            pin.set_value(LOW);
            pros::delay(200);
            return LOW;
        }
        else if(state == LOW){
            pin.set_value(HIGH);
            pros::delay(200);
            return HIGH;
        }
    }
    return state;
}

