#include "globals.hpp"

/*

Description:

This will be based off the heading of the field. We will modify
our autonomous scriptd based off the corner we are in.

There's 4 thingd we need in auton, we need to move in staright lines,
turn, run the intake, and run the clamp. We made 4 functions for these.
They work by knowing the distance per tick of a motor. I calculated this by: 
distPerTick = ((2 * wheelRadius) * gearRatio * Pi) / 1800;
We find out how far the wheel travels with one full rotation (in inches),
then we divide this by how many ticks. Since there is 
1800 ticks in a blue motor, we divide by 1800.

Same logic applies to the turning, we use the same distance, but apply
some geometry to it by implementing theta / 360 and rthe wheelbase.
we then figure out how many ticks based off degrees by: 
((theta / 360) * Pi * wheelBase) / distOneTick; 
We get this from the simplified equation.

The Intake works by timing how long it takes for a donut to make
it ot the top. This was the easiest and most simple way to do it,
as calculating distance for this was a bit overkill.

The pnuematics work by just setting it to the desired state wanted,
high or low. Meaning we cna clamp onto a goal during auton.


Pseudocode:

void moveLinear(distance, speed):
    //we know the distance travlled by one tick of motor
    //Used for driving forwards and backwards

    reset global position of motors (tare)
    get ticks requrired based off dist per 1 tick
    move motors that many ticks
    stop motors when reaches range of +- 5 of deisred number
    brake motors with coasting, for smoothness
    delay(100) //allows for brain to catch up

void moveAngular(angle, speed);
    //we know distance travlled by one tick of motor
    //useder for turning clockwise and counter clockwise

    reset flobal position of motors (tare)

    get ticks reqauired to turn based off formula
    move left motors forwards that many ticks  //allows us to turn in place
    move right motors backwards that many ticks
    stop motors when reaches range of +- 5 of desired ticks
    brake motors with coasting, for smoothness
    delay(100) //allows for brain to catch up

void autonIntake():
    move intake motors at certai speed
    wait for how long it takes to get to top
    stop moving intake motors

void setAutonPin(bool state, pros::adi:Port pin):
    //allows us to set the state of that pin during auton,
    //dont need to keep track of it as it's only used in auton
    pin.setValue(state)

*/


/*
path planning:

START BACKWARDS

bottom left and top right:
move backwards
clamp
intake
left 90
forward
intake



bottom right and top left
backwards
clamp
intake
right 90
forward
intake

*/

/*
    FRONT IS BLUE RIGHT, RED LEFT, 4 IN BACK
*/
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
    pros::delay(10000); // wait for intake to finish
}

void redGoalRush() {
    linearPID(-24);
    setAutonPin(LOW, clampPin); // Grabbed goal
    intakeMotors.move(127);
    linearPID(-20);

    angularPID(-90);
    linearPID(26);
    pros::delay(10000); // wait for intake to finish
}

void blueRingRush() {
    linearPID(-24);
    setAutonPin(LOW, clampPin); // Grabbed goal
    intakeMotors.move(127);
    linearPID(-20);

    angularPID(-90);
    linearPID(26);
    pros::delay(10000); // wait for intake to finish
}

void blueGoalRush() {
    linearPID(-24);
    setAutonPin(LOW, clampPin); // Grabbed goal
    intakeMotors.move(127);
    linearPID(-20);

    angularPID(90);
    linearPID(26);
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
 * @brief Skills Pathing
 * @attention facing direct North (Drivers Box South) infront of alliance wall stake
 * 
*/
void newAutonSkills() {
    // linearPID(8);
    // angularPID(180);
    // angularPID(-90);
    // linearPID(-24);
    // setAutonPin(LOW, clampPin); // Grabbed new goal (in the bottom left)
    // angularPID(180);
    // linearPID(24);
    // linearPID(8);
    // angularPID(180);
    // angularPID(-90);
    // linearPID(-24);
    // setAutonPin(LOW, clampPin); // Grabbed new goal (in the bottom left)
    // angularPID(180);
    // linearPID(24);

    linearPID(-10);
    angularPID(90);
    linearPID(-24);
    setAutonPin(LOW, clampPin); // Grabbed new goal (in the bottom left)
    // intakeMotors.move(114);
    linearPID(-6);
    angularPID(180); // facing to grab new rings

    linearPID(24);
    angularPID(-90);
    linearPID(24);


    pros::delay(1000000); 


}

/** 
 * @brief Skills Pathing
 * @attention facing direct North (Drivers Box South) infront of alliance wall stake
 * 
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
