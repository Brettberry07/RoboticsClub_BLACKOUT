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
as calculating distance for this was a bit overkill.acos

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

void topLeft(){

    driveTrainMove(-28, 70);
    setAutonPin(HIGH, clampPin);
    intakeMotors.move(75);
    driveTrainTurn(90, 30);
    driveTrainMove(28, 70);
    // autonIntake();
}

void topRight(){

    driveTrainMove(-28, 70);
    setAutonPin(HIGH, clampPin);
    intakeMotors.move(75);
    driveTrainTurn(-90, 30);
    driveTrainMove(28, 70);
    // autonIntake();
}

void bottomLeft(){

    driveTrainMove(-28, 70);
    setAutonPin(HIGH, clampPin);
    intakeMotors.move(75);
    driveTrainTurn(-90, 30);
    driveTrainMove(28, 70);
    // autonIntake();
}

void bottomRight(){

    driveTrainMove(-28, 70);
    setAutonPin(HIGH, clampPin);
    intakeMotors.move(75);
    driveTrainTurn(90, 30);
    driveTrainMove(28, 70);
    // autonIntake();
}

void testAuton(){
    // linearPID(24.0);
    angularPID(90);
    pros::delay(100000);

    // driveTrainMove(-28, 70);
    // setAutonPin(HIGH, clampPin);
    // autonIntake(1);
}

void autonSkills() {
    driveTrainMove(-4, 70);
    setAutonPin(HIGH, clampPin);
    intakeMotors.move(75);
    driveTrainMove(-4, 70);
    driveTrainTurn(180, 50);
    driveTrainMove(26, 70);
    driveTrainTurn(-90, 70);
    driveTrainMove(24, 70);
    driveTrainTurn(-90, 30);
    driveTrainMove(28, 70);
    pros::delay(500);
    driveTrainMove(10, 100);
    driveTrainTurn(-110, 30);
    driveTrainMove(-24, 90);
    setAutonPin(LOW, clampPin);
    driveTrainMove(12, 100);

    pros::delay(15000);
}
