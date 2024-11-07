#include "globals.hpp"

#define DRIVE_INTAKE_PIN 2
#define CLAMP_PIN 1

//Define all my global definitions like the motors and controllers

//drivetrain
pros::MotorGroup leftChassis({-1, -2});
pros::MotorGroup pneumaticsLeftChassis({-1,-2,-3});     //when pneumatics are on 
pros::MotorGroup rightChassis({9,10});
pros::MotorGroup pneumaticsRightChassis({9,10,8});   //pneumatics on
pros::MotorGroup driveTrainMotors({-1,-2,9,10});
bool isCurved = true;

//used for drivetrain and autonomous
const uint8_t wheelRadius = 3.25;                                //Radius of the wheel
const double distPerTick = ((2 * wheelRadius) * gearRatio * M_1_PI) / 1800; //This gives us are distance in inches
const double distOneTick = 0.0189;
const double wheelBase = 12.875;  //I need wheel base!
double gearRatio = 0.6;   //motor 36: Wheel: 60 360 rpm

//intake variables
pros::MotorGroup intakeMotors({3,-8});


//prot sensors (tri-port)
pros::IMU imuSensor(3);
pros::adi::Port driveIntakePin(DRIVE_INTAKE_PIN, pros::E_ADI_DIGITAL_OUT);
pros::adi::Port clampPin(CLAMP_PIN, pros::E_ADI_DIGITAL_OUT);

//pneumatics states
bool clampPneumaticsState = LOW;
bool driveOrIntakeState = LOW; //This is for the drive train and intake
                          //LOW = drivetrain, HIGH = intake

//controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

