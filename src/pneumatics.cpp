#include "globals.hpp"
#include "pneumatics.hpp"
#include "robot.hpp"

/*
================================================================================
PNEUMATICS - PSEUDOCODE
================================================================================

PURPOSE: Control pneumatic actuators (cylinders/pistons) for mechanisms like
         clamps, wings, or other binary (on/off) systems using digital ports

CLASS: Pneumatics
DEPENDENCIES: pros::Controller (controller_), pros::adi::Port (digital ports)

================================================================================

DRIVER CONTROL
--------------

bool toggle(state, button, pin):
    // Toggle pneumatic state when button is pressed
    // Returns new state for tracking in calling code
    if button is pressed:
        if current state is HIGH (extended):
            set pin to LOW (retract)
            display "ON" on controller screen
            delay 200ms (debounce)
            return LOW
        else if current state is LOW (retracted):
            set pin to HIGH (extend)
            display "OFF" on controller screen
            delay 200ms (debounce)
            return HIGH
    return state (no change if button not pressed)
    
    // Usage: clampState = toggle(clampState, L1, clampPin)


AUTONOMOUS CONTROL
------------------

void set(state, pin):
    // Directly set pneumatic state during autonomous
    // No button checking, immediate response
    set pin to state (HIGH or LOW)
    delay 150ms (allow pneumatic to actuate)
    
    // Usage: set(HIGH, clampPin) to extend clamp


LEGACY WRAPPERS (For Compatibility)
------------------------------------

bool switchState(state, button, pin):
    // Global function wrapper for toggle
    call getRobot().pneumatics.toggle(state, button, pin)
    return new state

void setAutonPin(state, pin):
    // Global function wrapper for autonomous control
    call getRobot().pneumatics.set(state, pin)

================================================================================
NOTES:
- HIGH = pneumatic extended (typically grabs/engages)
- LOW = pneumatic retracted (typically releases/disengages)
- 200ms debounce prevents rapid toggling from single button press
- Controller text shows current state for driver feedback
- Pin parameter passed by reference to modify actual hardware port
================================================================================
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

