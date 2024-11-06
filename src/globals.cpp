#include "globals.hpp"

#define DRIVE_INTAKE_PIN 2
#define CLAMP_PIN 1

//Define all my global definitions like the motors and controllers
pros::MotorGroup leftChassis({-1,-2});
pros::MotorGroup pneumaticsLeftChassis({-1,-2,-3});     //when pneumatics are on 
pros::MotorGroup rightChassis({9,10});
pros::MotorGroup pneumaticsRightChassis({9,10,8});   //pneumatics on
pros::MotorGroup intakeMotors({3,-8});
pros::MotorGroup driveTrainMotors({-1,-2,9,10});

pros::IMU imuSensor(3);

pros::adi::Port driveIntakePin(DRIVE_INTAKE_PIN, pros::E_ADI_DIGITAL_OUT);
pros::adi::Port clampPin(CLAMP_PIN, pros::E_ADI_DIGITAL_OUT);
bool driveTrainPneumaticsState = false;
bool clampPneumaticsState = false;

const uint8_t wheelRadius = 3.25;                                //Radius of the wheel
const double distPerTick = ((2*wheelRadius)*gearRatio*M_1_PI)/1800; //This gives us are distance in inches
const double distOneTick = 0.0189;
const double wheelBase = 15;  //I need wheel base!

double gearRatio = 0.6;   //motor 36: Wheel: 60 360 rpm

bool isCurved = false;
bool driveOrIntake = LOW; //This is for the drive train and intake
                          //LOW = drivetrain, HIGH = intake
pros::Controller master(pros::E_CONTROLLER_MASTER);

