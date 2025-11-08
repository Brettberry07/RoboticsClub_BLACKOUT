#include "globals.hpp"

#define DRIVE_INTAKE_PIN 2
#define CLAMP_PIN 1// A

#define LOW_INTAKE_PIN 17
#define MID_INTAKE_PIN 20
#define HIGH_INTAKE_PIN 15


// Define all global objects such as motors, controllers, and sensors.

// left chassis motors: ports 8, 6, 10 (reversed)
// right chassis motors: ports 9, 7, 5
// imu sensor: port 12
// vertical rotation sensor: port 1
// intake motors: ports 3, 1, 2

// Drivetrain
pros::MotorGroup leftChassis({-8,-6,-10});
pros::MotorGroup rightChassis({9,7,5});
pros::MotorGroup driveTrainMotors( {-8,-6,-10,9,7,5} );
bool isCurved = true;

// Drift compensation: multiplier for left side motors (e.g., 0.95 = 95% power to left)
// Tune this value to compensate for mechanical drift. 1.0 = no compensation.
double DRIFT_COMPENSATION = 0.95;

// Turn sensitivity: multiplier for turning input (e.g., 0.5 = 50% turn speed)
// Tune this value to adjust turn responsiveness. 1.0 = full sensitivity, 0.5 = half sensitivity.
double TURN_SENSITIVITY = 0.85;

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

// Tracking wheel constants (for rotation sensor on port 1)
const double trackingWheelDiameter = 2.0;  // 2 inch diameter tracking wheel
const double trackingWheelCircumference = M_PI * trackingWheelDiameter;  // ~6.283 inches per rotation

// Intake motors (three-stage system)
pros::Motor lowIntakeMotor(LOW_INTAKE_PIN);   // Bottom intake for picking up
pros::Motor midIntakeMotor(MID_INTAKE_PIN);   // Middle intake for transfer
pros::Motor highIntakeMotor(HIGH_INTAKE_PIN); // Top intake for scoring

// Ports and sensors (tri-port)
pros::IMU imuSensor(12);
pros::Rotation rotationSensor(1);  // Rotation sensor on port 1 (tracking wheel)
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
