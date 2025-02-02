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

//If a button is pressed, we toggle the ports state,
//We can swap between HIGH and LOW for pneumatics.
bool switchState(bool state, pros::controller_digital_e_t button, pros::adi::Port pin){
    /*
    Example:
        control clamp by seeing if the clamp is already on, or off
        when the button is pressed. We swap the state to the oppostie state
        that it is now. This allows us to grab onto mobile goals,
        and can be used to drop the mobile goals.
    */
    if(master.get_digital(button)){
        // state = LOW ? HIGH : LOW;     //if low, equals high, else equals low
        if(state == HIGH){
            pin.set_value(LOW);
            master.set_text(0,0,"ON");
            pros::delay(200);
            return LOW;
        }
        else if(state == LOW){
            pin.set_value(HIGH);
            master.set_text(0,0,"OFF ");
            pros::delay(200);
            return HIGH;
        }
    }
    return state;
}

// ---------------------------------------Used for autonomous---------------------------------------------------- //

/**
 * @brief Changes state of Clamp for mobile goals on back of robot
 * 
 * @param state The state of the Clamp (HIGH - On, LOW - Off)
*/
void setAutonPin(bool state, pros::adi::Port pin){
    pin.set_value(state);
    pros::delay(150);
}

