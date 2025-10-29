#include "globals.hpp"

/*

Description
-----------

Autonomous routines depend on field heading; we modify the sequence based on
the starting corner.

We need four capabilities in auton: move straight, turn, run the intake, and
operate the clamp. We provide four functions for these. They rely on the
distance traveled per encoder tick, computed as:
  distPerTick = ((2 * wheelRadius) * gearRatio * Pi) / 1800
This gives the distance of one full wheel rotation (in inches) divided by the
number of ticks (1800 for a blue motor).

Turning uses the same distance unit with additional geometry via theta/360 and
the wheelbase:
  ticks = ((theta / 360) * Pi * wheelBase) / distOneTick

Intake is timed based on how long a ring takes to reach the top—simple and
effective without computing travel distance.

Pneumatics are toggled to the desired state (HIGH/LOW) so we can clamp during
autonomous.


Pseudocode
----------

void moveLinear(distance, speed):
    // Distance per tick is known; used for forward/backward driving.
    tare global motor positions
    ticks = distance / distOneTick
    move motors to ticks
    wait until within ±5 ticks of target
    brake motors (coast) for smoothness
    delay(100)  // allow brain to catch up

void moveAngular(angle, speed):
    // Distance per tick known; used for clockwise/counterclockwise turns.
    tare global motor positions
    ticks = ((angle / 360) * Pi * wheelBase) / distOneTick
    left motors move +ticks; right motors move -ticks  // in-place turn
    wait until within ±5 ticks of target
    brake motors (coast) for smoothness
    delay(100)

void autonIntake():
    move intake motors at chosen speed
    wait long enough for ring to reach the top
    stop intake motors

void setAutonPin(bool state, pros::adi::Port pin):
    // Set the state of a pneumatics pin during auton only (no tracking needed).
    pin.set_value(state)

*/


/*
Path Planning (Notes)
---------------------

START BACKWARDS

Bottom Left and Top Right:
    - Move backwards
    - Clamp
    - Intake
    - Left 90
    - Forward
    - Intake

Bottom Right and Top Left:
    - Backwards
    - Clamp
    - Intake
    - Right 90
    - Forward
    - Intake

*/

/* FRONT IS BLUE RIGHT, RED LEFT, 4 IN BACK */
// void topLeft() {
//     driveTrainMove(-28, 70);
//     setAutonPin(HIGH, clampPin);
//     autonIntake();
//     driveTrainTurn(90, 30);
//     driveTrainMove(28, 70);
//     setAutonPin(HIGH, clampPin);
//     autonIntake();
//     autonIntake();
//     driveTrainTurn(90, 70);
//     driveTrainMove(15, 70);
//     driveTrainTurn(15, 30);
//     driveTrainMove(-32, 70);
//     driveTrainMove(17, 100);
//     pros::delay(15000);
// }

void redRingRush() {
    linearPID(-24);
    setAutonPin(LOW, clampPin); // Grabbed goal
    intakeMotors.move(127);
    linearPID(-20);

    angularPID(90);
    linearPID(26);

    pros::delay(300);
    linearPID(-12);

    pros::delay(10000); // wait for intake to finish
}

void redGoalRush() {
    linearPID(-24);
    setAutonPin(LOW, clampPin); // Grabbed goal
    intakeMotors.move(127);
    linearPID(-20);

    angularPID(-90);
    linearPID(26);

    pros::delay(300);
    linearPID(-12);

    pros::delay(10000); // wait for intake to finish
}

void blueRingRush() {
    linearPID(-24);
    setAutonPin(LOW, clampPin); // Grabbed goal
    intakeMotors.move(127);
    linearPID(-20);

    angularPID(-90);
    linearPID(26);
    pros::delay(300);
    linearPID(-12);

    pros::delay(10000); // wait for intake to finish
}

void blueGoalRush() {
    linearPID(-24);
    setAutonPin(LOW, clampPin); // Grabbed goal
    intakeMotors.move(127);
    linearPID(-20);

    angularPID(90);
    linearPID(26);

    pros::delay(300);
    linearPID(-12);
    
    pros::delay(10000); // wait for intake to finish
}



void topLeft(){


}

void topRight(){

}

void bottomLeft(){

}

void bottomRight(){

}

void testAuton(){
    // driveTrainMove(24.0);
    // driveTrainTurn(90, -10);

    // linearPID(-24);
    angularPID(45);
    angularPID(90);
    angularPID(135);
    angularPID(180);
    angularPID(225);
    angularPID(270);
    angularPID(315);
    angularPID(360);


    pros::delay(100000);

    // driveTrainMove(-28, 70);
    // setAutonPin(HIGH, clampPin);
    // autonIntake(1);
}

/** 
 * Skills Pathing
 * --------------
 * @attention Facing due North (Driver Box South) in front of alliance wall stake.
 */
void newAutonSkills() {
    linearPID(-8);
    angularPID(90);
    linearPID(-24);
    setAutonPin(LOW, clampPin); // Grabbed new goal (in the bottom left)
    intakeMotors.move(127);
    linearPID(-20);
    angularPID(180); // facing to grab new rings

    linearPID(24);
    linearPID(4);
    angularPID(-90);
    linearPID(24);
    linearPID(-2);
    
    linearPID(5);
    angularPID(0);
    linearPID(24);
    pros::delay(100);
    angularPID(200);
    linearPID(5);
    angularPID(200);
    linearPID(20);
    angularPID(90);
    linearPID(-24);
    setAutonPin(HIGH, clampPin);
    angularPID(960);
    

    pros::delay(1000000); 


}

/** 
 * Skills Pathing
 * --------------
 * @attention Facing due North (Driver Box South) in front of alliance wall stake.
 */
void autonSkills() { // our safe auton skills, gets 8 points
    linearPID(-5);
    pros::delay(500);
    // scoring preload
    setAutonPin(LOW, clampPin); // Grabbed new goal (in the bottom left)
    intakeMotors.move(114);

    //moving to corner
    linearPID(-32);
    angularPID(120);
    linearPID(-55); // in the corner now (bottom left)
    setAutonPin(HIGH, clampPin); // drop goal

    // going to next goal (in top left)
    linearPID(20 );
    angularPID(-120);
    linearPID(24);
    angularPID(-90);
    linearPID(-48);
    pros::delay(500);
    setAutonPin(LOW, clampPin); // Grabbed new goal

    // put in corner (top left corner)
    angularPID(-90);
    linearPID(24);
    angularPID(45);
    linearPID(-49);
    setAutonPin(HIGH, clampPin); // drop goal

    linearPID(49);
    pros::delay(1000000);
}
