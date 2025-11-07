#include "globals.hpp"
#include "robot.hpp"

/*
================================================================================
INTAKE - PSEUDOCODE
================================================================================

PURPOSE: Control intake motors for picking up and ejecting game pieces
         (rings/triballs) during both driver control and autonomous periods

CLASS: Intake
DEPENDENCIES: pros::MotorGroup (motors_), pros::Controller (controller_)

================================================================================

DRIVER CONTROL
--------------

void teleopControl():
    // Button-based intake control during driver period
    if R1 button pressed:
        run motors at -127 (reverse/outtake)
    else if R2 button pressed:
        run motors at 127 (forward/intake)
    else:
        stop motors (0 power)
    // R1 = outtake, R2 = intake, neither = stop


MANUAL CONTROL
--------------

void run(power):
    // Direct power control for intake motors
    clamp power to range [-127, 127]
    move motors at power
    // Positive = intake, negative = outtake


AUTONOMOUS HELPERS
------------------

void autonRunSeconds(seconds):
    // Timed intake for autonomous - runs for fixed duration
    run motors at 75 power
    wait (seconds * 1000) milliseconds
    stop motors (0 power)
    delay 100ms for settling


CONFIGURATION & TESTING
------------------------

void tare():
    // Reset intake encoder positions to zero
    reset motor positions to 0

void setVoltage(mV):
    // Direct voltage control in millivolts
    apply mV to motors

int getPosition():
    // Read current encoder position
    return motor position

void setBrakeMode(mode):
    // Set brake behavior (COAST/BRAKE/HOLD)
    set all motors to mode


LEGACY WRAPPERS (For Compatibility)
------------------------------------

void intake():
    // Global function wrapper for teleopControl
    call getRobot().intake.teleopControl()

void autonIntake(time):
    // Global function wrapper for autonomous intake
    call getRobot().intake.autonRunSeconds(time)

================================================================================
*/

// Get controller press, then move intake accordingly.

/**
 * @brief Controls the intake motors based on controller input.
 *
 * This function checks the state of the controller's digital buttons R1 and R2.
 * If R1 is pressed, the intake motors will run in reverse (outtake).
 * If R2 is pressed, the intake motors will run forward (intake).
 * If neither button is pressed, the intake motors will stop.
 */

// OOP implementation
void Intake::teleopControl(){
    if(controller_.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
        motors_.move(-127);
    } else if(controller_.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        motors_.move(127);
    } else {
        motors_.move(0);
    }
}

void Intake::run(int power){
    if (power > 127) power = 127;
    if (power < -127) power = -127;
    motors_.move(power);
}

void Intake::autonRunSeconds(int seconds){
    motors_.move(75);
    pros::delay(1000 * seconds);
    motors_.move(0);
    pros::delay(100);
}

// Legacy wrappers delegate to Robot OOP
void intake(){ getRobot().intake.teleopControl(); }

// ------------------------------------ Autonomous Helpers ------------------------------------ //

/*
Description:
    Used only during autonomous. Runs intake for a fixed time because it’s
    simpler to measure than exact travel distance and we lack sensors to detect
    when the ring reaches the top. Tested to determine a good time window.

Pseudocode:
    autonIntake():
            move intake motors
            wait a certain time
            stop moving motors
*/

// Run the intake for a specified duration (seconds) during autonomous.
void autonIntake(int time){
    getRobot().intake.autonRunSeconds(time);
}

// ---------------------------------------------------------------------------
// Migration / test helpers
// ---------------------------------------------------------------------------
void Intake::tare(){
    motors_.tare_position();
}

void Intake::setVoltage(int mV){
    motors_.move_voltage(mV);
}

int Intake::getPosition() const {
    return motors_.get_position();
}

void Intake::setBrakeMode(pros::motor_brake_mode_e_t mode) {
    motors_.set_brake_mode_all(mode);
}
