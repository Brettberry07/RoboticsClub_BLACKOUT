#include "globals.hpp"
#include "robot.hpp"

/*
================================================================================
INTAKE - PSEUDOCODE (THREE-STAGE SYSTEM)
================================================================================

PURPOSE: Control three-stage intake system for picking up, transferring, 
         and scoring game pieces at different heights

CLASS: Intake
DEPENDENCIES: pros::Motor (lowMotor_, midMotor_, highMotor_), pros::Controller

INTAKE MODES:
-------------
1. STOP         - All motors off
2. INTAKE       - Low motor forward (pick up from ground)
3. SCORE_LOW    - Low motor backward (outtake/score low)
4. SCORE_MID    - Mid forward + High forward (score middle goal)
5. SCORE_HIGH   - Mid forward + High backward (score high goal)
6. STORE        - Mid forward + High forward (store in robot)

================================================================================

DRIVER CONTROL
--------------

void teleopControl():
    // Button-based intake control during driver period
    if R2 button pressed:
        setMode(INTAKE)          // Pick up from ground
    else if R1 button pressed:
        setMode(SCORE_MID)       // Score middle
    else if L2 button pressed:
        setMode(SCORE_LOW)       // Score low (reverse intake)
    else if L1 button pressed:
        setMode(SCORE_HIGH)      // Score high
    else:
        setMode(STOP)            // Stop all motors


MODE CONTROL
------------

void setMode(mode):
    // Execute the specified intake mode
    switch mode:
        case STOP:
            low = 0, mid = 0, high = 0
        case INTAKE:
            low = 127 (forward), mid = 0, high = 0
        case SCORE_LOW:
            low = -127 (backward), mid = 0, high = 0
        case SCORE_MID:
            low = 0, mid = 127 (forward), high = 127 (forward)
        case SCORE_HIGH:
            low = 0, mid = 127 (forward), high = -127 (backward)
        case STORE:
            low = 0, mid = 127 (forward), high = 127 (forward)
    apply power to respective motors


INDIVIDUAL MOTOR CONTROL
-------------------------

void runLow(power):
    clamp power to [-127, 127]
    move low motor at power

void runMid(power):
    clamp power to [-127, 127]
    move mid motor at power

void runHigh(power):
    clamp power to [-127, 127]
    move high motor at power

void stopAll():
    stop all three motors


AUTONOMOUS HELPERS
------------------

void autonRunSeconds(mode, seconds):
    // Run a specific mode for fixed duration
    setMode(mode)
    wait (seconds * 1000) milliseconds
    stopAll()
    delay 100ms for settling


CONFIGURATION
-------------

void tare():
    reset all motor positions to 0

void setBrakeMode(mode):
    set all motors to brake mode (COAST/BRAKE/HOLD)

================================================================================
*/

// OOP implementation - Teleop control with multiple modes
void Intake::teleopControl(){
    if(controller_.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
        setMode(IntakeMode::INTAKE);
    } else if(controller_.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
        setMode(IntakeMode::SCORE_MID);
    } else if(controller_.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        setMode(IntakeMode::SCORE_LOW);
    } else if(controller_.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
        setMode(IntakeMode::SCORE_HIGH);
    } else {
        setMode(IntakeMode::STOP);
    }
}

void Intake::setMode(IntakeMode mode) {
    switch(mode) {
        case IntakeMode::STOP:
            lowMotor_.move(0);
            midMotor_.move(0);
            highMotor_.move(0);
            break;
            
        case IntakeMode::INTAKE:
            // Low intake forward to pick up from ground
            lowMotor_.move(127);
            midMotor_.move(0);
            highMotor_.move(0);
            break;
            
        case IntakeMode::SCORE_LOW:
            // Low intake backward to score low/outtake
            lowMotor_.move(-127);
            midMotor_.move(0);
            highMotor_.move(0);
            break;
            
        case IntakeMode::SCORE_MID:
            // Mid forward + High forward to score middle
            lowMotor_.move(0);
            midMotor_.move(127);
            highMotor_.move(127);
            break;
            
        case IntakeMode::SCORE_HIGH:
            // Mid forward + High backward to score high
            lowMotor_.move(0);
            midMotor_.move(127);
            highMotor_.move(-127);
            break;
            
        case IntakeMode::STORE:
            // Mid forward + High forward to store
            lowMotor_.move(0);
            midMotor_.move(127);
            highMotor_.move(127);
            break;
    }
}

void Intake::runLow(int power){
    if (power > 127) power = 127;
    if (power < -127) power = -127;
    lowMotor_.move(power);
}

void Intake::runMid(int power){
    if (power > 127) power = 127;
    if (power < -127) power = -127;
    midMotor_.move(power);
}

void Intake::runHigh(int power){
    if (power > 127) power = 127;
    if (power < -127) power = -127;
    highMotor_.move(power);
}

void Intake::stopAll(){
    lowMotor_.move(0);
    midMotor_.move(0);
    highMotor_.move(0);
}

void Intake::autonRunSeconds(IntakeMode mode, int seconds){
    setMode(mode);
    pros::delay(1000 * seconds);
    stopAll();
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
    getRobot().intake.autonRunSeconds(Intake::IntakeMode::INTAKE, time);
}

// ---------------------------------------------------------------------------
// Migration / test helpers
// ---------------------------------------------------------------------------
void Intake::tare(){
    lowMotor_.tare_position();
    midMotor_.tare_position();
    highMotor_.tare_position();
}

void Intake::setBrakeMode(pros::motor_brake_mode_e_t mode) {
    lowMotor_.set_brake_mode(mode);
    midMotor_.set_brake_mode(mode);
    highMotor_.set_brake_mode(mode);
}
