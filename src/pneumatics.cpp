#include "globals.hpp"
#include "pneumatics.hpp"
#include "robot.hpp"

/*
Description
-----------
General method to switch states (on/off). Takes the current state, the button
to toggle, and the tri-port the pneumatic is connected to.

We flip the current state, set the pin to the new value, and return the new
state so other files (e.g., drivetrain) can keep track of it.

Pseudocode
---------
bool switch_state(current_state, button, pin):
    new_state = (state is HIGH) ? LOW : HIGH
    set_value(pin, new_state)
    return new_state
*/

// If the button is pressed, toggle the port's state (HIGH/LOW) for pneumatics.
// OOP implementation
bool Pneumatics::toggle(bool state, pros::controller_digital_e_t button, pros::adi::Port& pin){
    if(controller_.get_digital(button)){
        if(state == HIGH){
            pin.set_value(LOW);
            controller_.set_text(0,0,"ON");
            pros::delay(200);
            return LOW;
        } else {
            pin.set_value(HIGH);
            controller_.set_text(0,0,"OFF ");
            pros::delay(200);
            return HIGH;
        }
    }
    return state;
}

// ------------------------------------ Autonomous Helpers ------------------------------------ //

/**
 * Clamp state (autonomous)
 * ------------------------
 * Change the state of the clamp for mobile goals on the back of the robot.
 *
 * @param state HIGH (on) or LOW (off).
 */
void Pneumatics::set(bool state, pros::adi::Port& pin){
    pin.set_value(state);
    pros::delay(150);
}

// Legacy wrappers delegate to Robot OOP
bool switchState(bool state, pros::controller_digital_e_t button, pros::adi::Port pin){
    return getRobot().pneumatics.toggle(state, button, pin);
}

void setAutonPin(bool state, pros::adi::Port pin){
    getRobot().pneumatics.set(state, pin);
}

