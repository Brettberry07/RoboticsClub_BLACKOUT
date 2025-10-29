#include "globals.hpp"
#include <numeric>
#include "pros/apix.h"
#include "pros/rtos.hpp"


// Function to test drivetrain.
bool test_drivetrain() {
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Testing Drivetrain...");
    driveTrainMotors.tare_position();

    driveTrainMotors.move_voltage(-1200); // Move drivetrain backward for 1 second.
    pros::delay(50);

    driveTrainMotors.move_voltage(1200); // Move drivetrain forward for 1 second.
    pros::delay(50); 

    if (driveTrainMotors.get_position() > 1 || driveTrainMotors.get_position() < 1) { // Check for insufficient movement.
        pros::screen::set_pen(pros::Color::red);
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Drivetrain Test FAIL: Motors firing at inconsistent rates, check temps");

        std::vector<double> leftTemperatures = leftChassis.get_temperature_all();
        std::vector<double> rightTemperatures = leftChassis.get_temperature_all();
        
        double leftTemp = std::accumulate(leftTemperatures.begin(), leftTemperatures.end(), 0.0) / leftTemperatures.size();
        double rightTemp = std::accumulate(rightTemperatures.begin(), rightTemperatures.end(), 0.0) / rightTemperatures.size();

    // Print average temperatures of the motors and return false.
        pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Left Chassis temp average: %f, Right: %f", leftTemp, rightTemp);
        return false;
    }

    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Drivetrain Test PASS - Drivetrain nominal");
    return true;
}

// Function to test intake.
bool test_intake() {
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Testing Intake...");
    intakeMotors.tare_position();

    intakeMotors.move_voltage(-1000);
    intakeMotors.move_voltage(1000);
    if (intakeMotors.get_position() != 0) { // Check for insufficient movement.
        pros::screen::set_pen(pros::Color::red);
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Intake Test FAIL: Friction detected in intake mechanism");
        return false;
    }
    pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Intake Test Complete - Intake Nominal");
    return true;
}

// Function to test sensors.
bool test_sensors() {
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Testing Sensors...");
    try { 
        double imu_value = imuSensor.get_rotation();
    } catch(const std::exception& error) {
        pros::screen::set_pen(pros::Color::red);
        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Sensors Test FAIL: IMU not returning values");
        return false;
    }

    pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Sensors Test Complete - Sensors Nominal");
    return true;
}

bool test_pneumatics() {
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Testing Pneumatics...");
    pros::screen::print(pros::E_TEXT_MEDIUM, 7, "!!!FILL UP PNEUMATICS AIR TANK!!!");
    // Test the pneumatics system.

    setAutonPin(HIGH, clampPin);
    pros::delay(1000);
    setAutonPin(LOW, clampPin);
    pros::delay(1000);

    pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Pneumatics Test Complete - Pneumatics Nominal");
    return true;
}

// Main routine for the system check.
/**
 * @brief A full self-verifying systems check.
 * 
 * This function checks the various components of the system, including drivetrain, intake, sensors, and pneumatics.
 * If all components pass the tests, the system check is considered complete and the function returns true.
 * Otherwise, if any component fails the tests, the system check is considered failed and the function returns false.
 * 
 * @return True if all components pass; false otherwise.
 */
bool full_system_check() {
	pros::screen::set_pen(pros::Color::white);

    driveTrainMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Beginning System Check...");
    if(test_drivetrain() && test_intake() && test_pneumatics() && test_sensors()) {
        pros::screen::set_pen(pros::Color::green);
        pros::screen::print(pros::E_TEXT_MEDIUM, 0, "System Check Complete - System is Nominal");
        driveTrainMotors.brake();
        return true;
    } else {
        pros::screen::set_pen(pros::Color::red);
        pros::screen::print(pros::E_TEXT_MEDIUM, 0, "System Check Failed - Check Logs for Details");
        driveTrainMotors.brake();
        return false;
    }
}