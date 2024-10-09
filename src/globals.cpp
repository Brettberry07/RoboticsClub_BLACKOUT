#include "globals.hpp"

#define PNEUMATICS_PIN 2

//Define all my global definitions like the motors and controllers
pros::MotorGroup leftChassis({-1,-2});
pros::MotorGroup pneumaticsLeftChassis({-1,-2,-3});     //when pneumatics are on 
pros::MotorGroup rightChassis({9,10});
pros::MotorGroup pneumaticsRightChassis({9,10,11});
pros::MotorGroup intake({-12,13});

pros::IMU imuSensor(3);
pros::adi::DigitalOut pneumatics(PNEUMATICS_PIN,LOW);
double gearRatio = 0.6;   //motor 36: Wheel: 60 600 rpm, so 360

bool isCurved = false;
bool driveOrIntake = LOW; //This is for the drive train and intake
                          //LOW = drivetrain, HIGH = intake
pros::Controller master(pros::E_CONTROLLER_MASTER);