#include "globals.hpp"

/*
This will be based off the heading of the field 
*/

void testAuton(){
    driveTrainMove(16, 60);
    pros::delay(500);
    driveTrainMove(16, 60);
    pros::delay(500);
    driveTrainMove(16, 60);
    driveTrainMotors.brake();
    // driveTrainTurn(90, 60);

    pros::delay(100000000);
}