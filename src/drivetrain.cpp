#include "globals.hpp"
/**
 * =============================================================================
 * Drivetrain Controls Overview
 * =============================================================================
 *
 * Purpose
 * -------
 * Provides multiple driver-control schemes (Tank, Split-Arcade, One-Stick
 * Arcade) and optional input shaping via a cubic sensitivity curve for
 * precision vs. speed. The code is written to keep all original explanatory
 * comments while presenting them in a clean, consistent format.
 *
 * Notes
 * -----
 * - Drive schemes supported:
 *   - Tank: Left stick Y controls left side; Right stick Y controls right side.
 *   - Split: Left stick Y is forward/back; Right stick X is turning.
 *   - Arcade: Left stick Y is forward/back; Left stick X is turning.
 * - Sensitivity Curve:
 *   - A cubic curve can be applied to inputs for finer low-speed control.
 *   - Toggle is handled by checkCurveInput().
 *
 * Pseudocode (original intent)
 * ----------------------------
 * drivetrain(drivescheme):
 *   if tank drive      -> tankDrive()
 *   else if split      -> splitDrive()
 *   else if arcade     -> arcadeDrive()
 *   else               -> tankDrive()  // default for redundancy
 *
 * tankDrive():
 *   read both sticks' Y axes;
 *   if using cubic curve: apply cubicCurve() to each axis; else apply directly.
 *
 * cubicCurve(controller_axis):
 *   return controller_axis^3 / 127^2
 *
 * checkCurveInput():
 *   on button press, toggle between curved and linear drive.
 *
 * splitDrive():
 *   power = left stick Y; turn = right stick X; apply (power ± turn) to sides.
 *
 * arcadeDrive():
 *   power = left stick Y; turn = left stick X; apply (power ± turn) to sides.
 * =============================================================================
 */

// Access multiple driving methods quickly.
// We plan to use Tank drive; default falls back to Tank for redundancy.
void driveTrain(char driveScheme, bool isCurved){
    checkCurveInput();      // Check whether we are using curved (cubic) input.
    switch(driveScheme){
        case 't':
            tankDrive();
            break;
        case 's':
            splitDrive();
            break;
        case 'a':
            arcadeDrive();
            break;
        default:
            tankDrive();
            break;
    }
}

// Tank drive controls.
void tankDrive(){
    // Read left/right stick Y and apply directly (linear response).
    leftChassis.move(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
    rightChassis.move(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
}

void tankDriveCubic(){
    // Apply cubic curve for precision driving (slower initial response).
    leftChassis.move(cubicCurve(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)));
    rightChassis.move(cubicCurve(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)));
}

// Split drive: forward/back on left Y; turning on right X.
void splitDrive(){
    int power = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    leftChassis.move(power+turn);
    rightChassis.move(power-turn);
}

// One-stick arcade: forward/back on left Y; turning on left X.
void arcadeDrive(){
    int power = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
    leftChassis.move(power+turn);
    rightChassis.move(power-turn);
}

// Sensitivity curve helper.
int cubicCurve(int controllerInput){
    // Get percentage of controller input, cube that value, then return as 127-range integer.
    return ((controllerInput*controllerInput*controllerInput)/(127*127));

}

void checkCurveInput(){
    static bool prevPressed = false;
    bool pressed = master.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
    if (pressed && !prevPressed) {
        isCurved = !isCurved; // Toggle on rising edge.
    }
    prevPressed = pressed;
}


// ------------------------------------ Autonomous Helpers ------------------------------------ //

// Move forward/backward by a distance (in inches).
// Negative velocity moves backward; positive moves forward.
void driveTrainMove(double dist, int velocity){
    driveTrainMotors.tare_position();

    const double ticks = dist / distOneTick; // Required encoder counts.

    driveTrainMotors.move_relative(ticks, velocity);

    while (!((driveTrainMotors.get_position() < ticks + 5) && (driveTrainMotors.get_position() > ticks - 5))) {
        // Continue until motors are within ±5 ticks of the desired position.
        // We must wait, otherwise code would advance to the next function prematurely.
        pros::delay(2);
    }
    
    driveTrainMotors.brake();
    pros::delay(150);
}

// Turn in place by a specified angle (degrees).
// Negative velocity = counterclockwise; positive = clockwise.
void driveTrainTurn(double theta, int velocity){
    rightChassis.tare_position();
    leftChassis.tare_position();

    // Compute required ticks for the desired angle.
    // Formula:
    //   ticks = ((theta / 360) * π * wheelBase) / distOneTick
    // where wheelBase is the distance between left and right wheels.

    const double ticks = ((theta / 360.0) * M_PI * wheelBase) / distOneTick;

    rightChassis.move_relative(-ticks, velocity);
    leftChassis.move_relative(ticks, velocity);

    while (!((rightChassis.get_position() > ticks - 5) && (rightChassis.get_position() < ticks + 5) &&
             (leftChassis.get_position()  > ticks - 5) && (leftChassis.get_position()  < ticks + 5))) {
        pros::delay(2);
    }

    rightChassis.brake();
    leftChassis.brake();
    pros::delay(150);
}
