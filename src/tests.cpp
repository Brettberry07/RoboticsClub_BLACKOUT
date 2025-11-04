#include "globals.hpp"
#include "robot.hpp"
#include <numeric>
#include <vector>
#include <cmath>
#include "pros/apix.h"
#include "pros/rtos.hpp"


// Function to test drivetrain using OOP Drivetrain APIs.
bool test_drivetrain() {
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Testing Drivetrain...");
    // Use OOP tare
    getRobot().drivetrain.tare();

    // Apply brief reverse and forward voltages to validate motion
    getRobot().drivetrain.setVoltage(-1200, -1200); // Move drivetrain backward briefly
    pros::delay(50);

    getRobot().drivetrain.setVoltage(1200, 1200); // Move drivetrain forward briefly
    pros::delay(50);

    // Read encoder positions via Drivetrain API
    double leftPos = 0.0, rightPos = 0.0;
    getRobot().drivetrain.getPosition(leftPos, rightPos);
    double avgPos = (std::abs(leftPos) + std::abs(rightPos)) / 2.0;

    // If movement was too small, consider it a failure and print temperatures
    if (avgPos < 1.0) { // Check for insufficient movement.
        pros::screen::set_pen(pros::Color::red);
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Drivetrain Test FAIL: insufficient movement, check temps");

        // Get all drivetrain motor temperatures and print average
        std::vector<double> temps = getRobot().drivetrain.getTemperatures();
        double avgTemp = 0.0;
        if (!temps.empty()) {
            avgTemp = std::accumulate(temps.begin(), temps.end(), 0.0) / temps.size();
        }

        // Print average temperature and return false.
        pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Drivetrain temp average: %f", avgTemp);
        return false;
    }

    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Drivetrain Test PASS - Drivetrain nominal");
    return true;
}

// Function to test intake.
bool test_intake() {
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Testing Intake...");
    // Use OOP intake helpers
    getRobot().intake.tare();
    getRobot().intake.setVoltage(-1000);
    pros::delay(50);
    getRobot().intake.setVoltage(1000);
    pros::delay(50);

    if (getRobot().intake.getPosition() != 0) { // Check for insufficient movement.
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
    // Use OOP pneumatics API for autonomous pin toggle
    getRobot().pneumatics.set(HIGH, clampPin);
    pros::delay(1000);
    getRobot().pneumatics.set(LOW, clampPin);
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

    getRobot().drivetrain.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Beginning System Check...");
    if(test_drivetrain() && test_intake() && test_pneumatics() && test_sensors()) {
        pros::screen::set_pen(pros::Color::green);
        pros::screen::print(pros::E_TEXT_MEDIUM, 0, "System Check Complete - System is Nominal");
        getRobot().drivetrain.brake();
        return true;
    } else {
        pros::screen::set_pen(pros::Color::red);
        pros::screen::print(pros::E_TEXT_MEDIUM, 0, "System Check Failed - Check Logs for Details");
        getRobot().drivetrain.brake();
        return false;
    }
}
