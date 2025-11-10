#include "globals.hpp"
#include "robot.hpp"
#include "pidController.hpp"

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

}

void redGoalRush() {

}

void blueRingRush() {
}

void blueGoalRush() {
}



void topLeft(){ // red left
    linearPID(-36);
    getRobot().intake.setMode(Intake::IntakeMode::INTAKE);
    pros::delay(200);
    getRobot().intake.setMode(Intake::IntakeMode::SCORE_MID);
    // // Top Left autonomous routine (mirrors bottomRight)
    // // Symmetrical field: red left = blue right flipped
    
    // // Start intake to grab ring while moving
    // getRobot().intake.setMode(Intake::IntakeMode::INTAKE);
    
    // linearPID(25);  // Move forward 26 inches to pick up ring
    // pros::delay(100);  // Brief pause to ensure ring is secured

    // angularPID(90);  // Turn LEFT 90 degrees to face goal
    // pros::delay(100);

    // linearPID(14);  // Move forward to goal
    // linearPID(-6);
    // getRobot().intake.stopAll();
    // getRobot().intake.setMode(Intake::IntakeMode::SCORE_LOW);
    // pros::delay(100);

    // getRobot().intake.stopAll();

    // linearPID(-10);  // Backup slightly

    // angularPID(45);  // Turn LEFT 45 degrees
    // pros::delay(100);

    // linearPID(38);
    // pros::delay(100);

    // angularPID(-135);  // Turn to face ring
    // pros::delay(100);

    // getRobot().intake.setMode(Intake::IntakeMode::SCORE_LOW);
    // pros::delay(200);
    // getRobot().intake.setMode(Intake::IntakeMode::INTAKE);

    // linearPID(16);
    // pros::delay(100);
    // linearPID(16);
    // pros::delay(100);

    // // Stop intake
    // getRobot().intake.stopAll();

    // angularPID(-45);  // Turn RIGHT 45 degrees
    // pros::delay(100);

    // angularPID(-175);  // Turn to face goal
    // pros::delay(100);

    // linearPID(-16.5);
    // pros::delay(100);

    // // getRobot().intake.setMode(Intake::IntakeMode::SCORE_LOW);
    // // pros::delay(200);
    // getRobot().intake.setMode(Intake::IntakeMode::INTAKE);
    // pros::delay(300);
    // // Score middle - activate scoring mechanism
    // getRobot().intake.setMode(Intake::IntakeMode::SCORE_MID);
    // pros::delay(500);  // Run scoring for 0.5 seconds

    // getRobot().intake.setMode(Intake::IntakeMode::SCORE_LOW);
    // pros::delay(200);
    // getRobot().intake.setMode(Intake::IntakeMode::INTAKE);
    // pros::delay(300);
    // // Score middle - activate scoring mechanism
    // getRobot().intake.setMode(Intake::IntakeMode::SCORE_MID);
    // pros::delay(500);  // Run scoring for 0.5 seconds

    // getRobot().intake.setMode(Intake::IntakeMode::SCORE_LOW);
    // pros::delay(200);
    // getRobot().intake.setMode(Intake::IntakeMode::INTAKE);
    // pros::delay(300);
    // // Score middle - activate scoring mechanism
    // getRobot().intake.setMode(Intake::IntakeMode::SCORE_MID);
    // pros::delay(500);  // Run scoring for 0.5 seconds
}

void topRight(){ // red right
    // Top Right autonomous routine (same as bottomLeft)
    // Symmetrical field: red right = blue left (same side)
    
    // Start intake to grab ring while moving
    getRobot().intake.setMode(Intake::IntakeMode::INTAKE);
    
    linearPID(26);  // Move forward 26 inches to pick up ring
    pros::delay(100);  // Brief pause to ensure ring is secured

    angularPID(-90);  // Turn RIGHT 90 degrees to face goal
    pros::delay(100);

    linearPID(14);  // Move forward to goal
    linearPID(-6);
    getRobot().intake.stopAll();
    getRobot().intake.setMode(Intake::IntakeMode::SCORE_LOW);
    pros::delay(100);

    getRobot().intake.stopAll();

    linearPID(-10);  // Backup slightly

    angularPID(-45);  // Turn RIGHT 45 degrees
    pros::delay(100);

    linearPID(38);
    pros::delay(100);

    angularPID(150);  // Turn to face ring
    pros::delay(100);

    getRobot().intake.setMode(Intake::IntakeMode::SCORE_LOW);
    pros::delay(200);
    getRobot().intake.setMode(Intake::IntakeMode::INTAKE);

    // linearPID(32);
    // pros::delay(200);
    linearPID(18);
    pros::delay(75);
    linearPID(14);
    pros::delay(75);

    angularPID(45);  // Turn LEFT 45 degrees
    pros::delay(100);

    angularPID(176);  // Turn to face goal
    pros::delay(100);

    linearPID(-14.5);
    pros::delay(100);


    getRobot().intake.setMode(Intake::IntakeMode::INTAKE);
    pros::delay(300);
    // Score middle - activate scoring mechanism
    getRobot().intake.setMode(Intake::IntakeMode::SCORE_MID);
    pros::delay(500);  // Run scoring for 0.5 seconds

    getRobot().intake.setMode(Intake::IntakeMode::SCORE_LOW);
    pros::delay(200);
    getRobot().intake.setMode(Intake::IntakeMode::INTAKE);
    pros::delay(300);
    // Score middle - activate scoring mechanism
    getRobot().intake.setMode(Intake::IntakeMode::SCORE_MID);
    pros::delay(500);  // Run scoring for 0.5 seconds

    getRobot().intake.setMode(Intake::IntakeMode::SCORE_LOW);
    pros::delay(200);
    getRobot().intake.setMode(Intake::IntakeMode::INTAKE);
    pros::delay(300);
    // Score middle - activate scoring mechanism
    getRobot().intake.setMode(Intake::IntakeMode::SCORE_MID);
    pros::delay(500);  // Run scoring for 0.5 seconds
}

void bottomLeft(){ // blue left
    // Bottom Left autonomous routine
    // Path: Forward to pick up ring, turn right, forward to goal, score, turn around, backup to score mid
    
    // Start intake to grab ring while moving
    getRobot().intake.setMode(Intake::IntakeMode::INTAKE);
    
    linearPID(26);  // Move forward 24 inches to pick up ring
    pros::delay(100);  // Brief pause to ensure ring is secured

    angularPID(-90);  // Turn right 90 degrees to face goal
    pros::delay(100);

    linearPID(14);  // Mo ve forward to goal
    linearPID(-6);
    getRobot().intake.stopAll();
    getRobot().intake.setMode(Intake::IntakeMode::SCORE_LOW);
    pros::delay(100);

    getRobot().intake.stopAll();

    linearPID(-10);  // Backup slightly

    angularPID(-45);
    pros::delay(100);

    linearPID(38);
    pros::delay(100);

    angularPID(128);
    pros::delay(100);

    getRobot().intake.setMode(Intake::IntakeMode::SCORE_LOW);
    pros::delay(200);
    getRobot().intake.setMode(Intake::IntakeMode::INTAKE);

    linearPID(29);
    pros::delay(200);

    // Stop intake
    getRobot().intake.stopAll();


    angularPID(45);
    pros::delay(100);

    angularPID(175);
    pros::delay(100);

    linearPID(-16.5);
    pros::delay(100);

    getRobot().intake.setMode(Intake::IntakeMode::SCORE_LOW);
    pros::delay(200);
    getRobot().intake.setMode(Intake::IntakeMode::INTAKE);
    pros::delay(300);
    // Score middle - activate scoring mechanism
    getRobot().intake.setMode(Intake::IntakeMode::SCORE_MID);
    pros::delay(500);  // Run scoring for 1.5 seconds

    getRobot().intake.setMode(Intake::IntakeMode::SCORE_LOW);
    pros::delay(200);
    getRobot().intake.setMode(Intake::IntakeMode::INTAKE);
    pros::delay(300);
    // Score middle - activate scoring mechanism
    getRobot().intake.setMode(Intake::IntakeMode::SCORE_MID);
    pros::delay(500);  // Run scoring for 1.5 seconds

    getRobot().intake.setMode(Intake::IntakeMode::SCORE_LOW);
    pros::delay(200);
    getRobot().intake.setMode(Intake::IntakeMode::INTAKE);
    pros::delay(300);
    // Score middle - activate scoring mechanism
    getRobot().intake.setMode(Intake::IntakeMode::SCORE_MID);
    pros::delay(500);  // Run scoring for 1.5 seconds
}

void bottomRight(){ // blue right
    // Bottom Right autonomous routine (mirrors bottomLeft)
    // Symmetrical field: blue right turns left instead of right
    
    // Start intake to grab ring while moving
    getRobot().intake.setMode(Intake::IntakeMode::INTAKE);
      
    linearPID(26);  // Move forward 26 inches to pick up ring
    pros::delay(100);  // Brief pause to ensure ring is secured

    angularPID(90);  // Turn LEFT 90 degrees to face goal (mirrored)
    pros::delay(100);

    linearPID(14);  // Move forward to goal
    linearPID(-6);
    getRobot().intake.stopAll();
    getRobot().intake.setMode(Intake::IntakeMode::SCORE_LOW);
    pros::delay(100);

    getRobot().intake.stopAll();

    linearPID(-10);  // Backup slightly

    angularPID(45);  // Turn LEFT 45 degrees (mirrored)
    pros::delay(100);

    linearPID(38);
    pros::delay(100);

    angularPID(-128);  // Turn to face ring (mirrored)
    pros::delay(100);

    getRobot().intake.setMode(Intake::IntakeMode::SCORE_LOW);
    pros::delay(200);
    getRobot().intake.setMode(Intake::IntakeMode::INTAKE);

    linearPID(29);
    pros::delay(200);

    // Stop intake
    getRobot().intake.stopAll();

    angularPID(-45);  // Turn RIGHT 45 degrees (mirrored)
    pros::delay(100);

    angularPID(-175);  // Turn to face goal (mirrored)
    pros::delay(100);

    linearPID(-16.5);
    pros::delay(100);

    getRobot().intake.setMode(Intake::IntakeMode::SCORE_LOW);
    pros::delay(200);
    getRobot().intake.setMode(Intake::IntakeMode::INTAKE);
    pros::delay(300);
    // Score middle - activate scoring mechanism
    getRobot().intake.setMode(Intake::IntakeMode::SCORE_MID);
    pros::delay(500);  // Run scoring for 0.5 seconds

    getRobot().intake.setMode(Intake::IntakeMode::SCORE_LOW);
    pros::delay(200);
    getRobot().intake.setMode(Intake::IntakeMode::INTAKE);
    pros::delay(300);
    // Score middle - activate scoring mechanism
    getRobot().intake.setMode(Intake::IntakeMode::SCORE_MID);
    pros::delay(500);  // Run scoring for 0.5 seconds

    getRobot().intake.setMode(Intake::IntakeMode::SCORE_LOW);
    pros::delay(200);
    getRobot().intake.setMode(Intake::IntakeMode::INTAKE);
    pros::delay(300);
    // Score middle - activate scoring mechanism
    getRobot().intake.setMode(Intake::IntakeMode::SCORE_MID);
    pros::delay(500);  // Run scoring for 0.5 seconds
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
    getRobot().intake.setMode(Intake::IntakeMode::INTAKE);
    linearPID(38);
    pros::delay(800);

    angularPID(-48);
    linearPID(14);

    getRobot().intake.stopAll();
    pros::delay(100);
    getRobot().intake.setMode(Intake::IntakeMode::SCORE_LOW);

    /*angularPID(105);
    linearPID(31);
    pros::delay(100);
    
    getRobot().intake.stopAll();
    angularPID(-60);
    linearPID(15);

    pros::delay(100);
    getRobot().intake.setMode(Intake::IntakeMode::SCORE_HIGH);*/

    /*angularPID(72.5);
    setAutonPin(HIGH, clampPin);
    pros::delay(200);
    linearPID(16);
    pros::delay(1000);
    getRobot().intake.stopAll();
    linearPID(-16);
    pros::delay(100);
    setAutonPin(LOW, clampPin);*/

    /*linearPID(-22);
    angularPID(180);
    linearPID(22);
    setAutonPin(HIGH, clampPin);
    pros::delay(200);
    getRobot().intake.setMode(Intake::IntakeMode::INTAKE);
    pros::delay(900);
    getRobot().intake.stopAll();
    linearPID(-5);
    angularPID(135);
    linearPID(35);
    pros::delay(200);
    getRobot().intake.setMode(Intake::IntakeMode::SCORE_MID);
    pros::delay(1000);*/
}

/** 
 * Skills Pathing
 * --------------
 * @attention Facing due North (Driver Box South) in front of alliance wall stake.
 */
void autonSkills() { // our safe auton skills, gets 8 points
    
}
