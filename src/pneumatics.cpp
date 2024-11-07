#include "globals.hpp"
#include "pneumatics.hpp"

/*
DESCRIPTION:
    A general method to switch states (on or off)
    we take the current state, the button press to switch them,
    and the port that the ada tri-point-wire is connected to.

    We then swap the current state to the opposite,
    set that pin to the new value,
    return the new value to the current state so we can
    keep track of the state in other files (like drivetrain).

PSEUDOCODE:
    bool switch_state(current_state, button, pin):
        new state = if state is high, now low, else now high
        set_value of pin to new state
        return the new state
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

// ---------------------------------------Used for autonomous---------------------------------------------------- //

//set to a value I need in autonoumous
bool setAutonPin(bool state, pros::adi::Port pin){
    pin.set_value(state);
}

