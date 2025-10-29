#include "globals.hpp"

/*
Description
-----------
Move the intake motors within the range [-127, 127]. We typically use ±100 to
avoid spinning too fast. If the first button is pressed, run intake forward; if
the other button is pressed, reverse (outtake). Otherwise, stop the motors.

Code
----
if button_pressed == left_bumper_one:
    move intake motors backward (100)
elif button_pressed == left_bumper_two:
    move intake motors forward (100)
else:
    stop moving motors

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

void intake(){
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){  //outake
        intakeMotors.move(-127); // 110 works
    }
    else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){  //intake
        intakeMotors.move(127);
    }
    else{
        intakeMotors.move(0);
    }
}

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
    intakeMotors.move(75);
    pros::delay(1000*time);
    intakeMotors.move(0);
    pros::delay(100);
}
