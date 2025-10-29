#include "globals.hpp"

#define DRIVE_INTAKE_PIN 2
#define CLAMP_PIN 1

//Define all my global definitions like the motors and controllers

//drivetrain
pros::MotorGroup leftChassis({-1, -2, -3});
pros::MotorGroup pneumaticsLeftChassis({-1,-2,-3});     //when pneumatics are on 
pros::MotorGroup rightChassis({7,10, 8});
pros::MotorGroup pneumaticsRightChassis({7,10,8});   //pneumatics on
pros::MotorGroup driveTrainMotors( {-1, -2, -3, 7, 10, 8} );
bool isCurved = true;

//used for drivetrain and autonomous
const double wheelRadius = 3.25;     // Radius of the wheel (inches)

/**
 * @brief Calculates the distance traveled per encoder tick.
 *
 * This constant represents the distance traveled per tick of the encoder, 
 * calculated using the wheel radius, gear ratio, and a constant factor 
 * (π) to convert the result into inches.
 *
 * @note The formula used is: (2 * π * wheelRadius * gearRatio) / 1800
 * where:
 * - wheelRadius: The radius of the wheel.
 * - gearRatio: The gear ratio of the system.
 * - M_1_PI: The reciprocal of PI (1/PI).
 * - 1800: The number of encoder ticks per revolution.
 */
// Define gear ratio BEFORE using it to compute distances
double gearRatio = 0.6;   // motor 36 : wheel 60 (example)
const double distPerTick = (2.0 * M_PI * wheelRadius * gearRatio) / 1800.0; // inches per encoder tick
const double distOneTick = distPerTick; // keep single source of truth
const double wheelBase = 12.875; // 12.875 in.

//intake variables
pros::MotorGroup intakeMotors({-14,5});

//prot sensors (tri-port)
pros::IMU imuSensor(13);
pros::adi::Port driveIntakePin(DRIVE_INTAKE_PIN, pros::E_ADI_DIGITAL_OUT);
pros::adi::Port clampPin(CLAMP_PIN, pros::E_ADI_DIGITAL_OUT);

//pneumatics states
bool clampPneumaticsState = LOW;
bool driveOrIntakeState = LOW; //This is for the drive train and intake
                              //LOW = drivetrain, HIGH = intake

//auton selector variables
bool autonSelected = false;
char autonID;

//controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

//screen
pros::screen_touch_status_s_t status;

//odom variables
double globalHeading = 0;
double globalPos[2] = {0.0, 0.0};  //holds x and y values
