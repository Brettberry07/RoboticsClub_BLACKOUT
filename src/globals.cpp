#include "globals.hpp"

#define DRIVE_INTAKE_PIN 2
#define CLAMP_PIN 1

// Define all global objects such as motors, controllers, and sensors.

// Drivetrain
pros::MotorGroup leftChassis({-8, -9, -10});
pros::MotorGroup rightChassis({11, 12, 13});
pros::MotorGroup driveTrainMotors( {-8, -9, -10, 11, 12, 13} );
bool isCurved = true;

// Used for drivetrain and autonomous.
const double wheelRadius = 3.25;     // Wheel radius (inches).

/**
 * @brief Distance traveled per encoder tick.
 *
 * Represents the distance traveled per encoder tick. Computed using wheel
 * radius, gear ratio, and π to convert to inches.
 *
 * @note The formula used is: (2 * π * wheelRadius * gearRatio) / 1800
 * where:
 * - wheelRadius: Wheel radius (inches).
 * - gearRatio: System gear ratio.
 * - 1800: Ticks per revolution for the selected cartridge.
 */
// Define gear ratio BEFORE using it to compute distances.
double gearRatio = 0.375;   // Example: motor 36 : wheel 60.
const double distPerTick = (2.0 * M_PI * wheelRadius * gearRatio) / 1800.0; // Inches per encoder tick.
const double distOneTick = distPerTick; // Single source of truth.
const double wheelBase = 12.875; // Wheelbase (inches).

// Intake
pros::MotorGroup intakeMotors({-14,5});

// Ports and sensors (tri-port)
pros::IMU imuSensor(5);
pros::adi::Port driveIntakePin(DRIVE_INTAKE_PIN, pros::E_ADI_DIGITAL_OUT);
pros::adi::Port clampPin(CLAMP_PIN, pros::E_ADI_DIGITAL_OUT);

// Pneumatics state
bool clampPneumaticsState = LOW;

// Autonomous selector
bool autonSelected = false;
char autonID;

// Controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

// Screen/touch
pros::screen_touch_status_s_t status;

// Odometry
double globalHeading = 0;
double globalPos[2] = {0.0, 0.0};  // Holds X and Y values.
