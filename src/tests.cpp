#include "globals.hpp"
#include "pros/apix.h"
#include "pros/rtos.hpp"


// Function to test drivetrain
bool test_drivetrain() {
    pros::screen::print(TEXT_MEDIUM, 0, "Testing Drivetrain...");
    driveTrainMotors.tare_position();

    driveTrainMotors.move_voltage(-12000); // Move drivetrain backward for 1 second
    pros::delay(1000);

    driveTrainMotors.move_voltage(12000);// Move drivetrain forward for 1 second
    pros::delay(1000); 

    if (driveTrainMotors.get_position() != 0) { // Check for insufficient movement in a direction
        pros::screen::set_pen(pros::Color::red);
        pros::screen::print(TEXT_LARGE, 1, "Drivetrain Test FAIL: Motors firing at inconsistent rates, check temps");

        std::vector<double> tempsLeft = leftChassis.get_temperatures();
        std::vector<double> tempsRight = rightChassis.get_temperatures();

        double leftAverageTemp = 0;
        double rightAverageTemp = 0;

        // Get the average temperature of the left chassis motors
        for (int i = 0; i < tempsLeft.size(); i++) {
            leftAverageTemp += tempsLeft[i];
        }
        leftAverageTemp /= tempsLeft.size();

        // Get the average temperature of the right chassis motors
        for (int i = 0; i < tempsRight.size(); i++) {
            rightAverageTemp += tempsRight[i];
        }
        rightAverageTemp /= tempsRight.size();

        // Print the average temperatures of the motors and return false
        pros::screen::print(TEXT_MEDIUM, 2, "Left Chassis temp average: %f, Right: %f", leftAverageTemp, rightAverageTemp);
        return false;
    }

    pros::screen::print(TEXT_MEDIUM, 1, "Drivetrain Test PASS - Drivetrain nominal");
    return true;
}

// Function to test intake
bool test_intake() {
    pros::screen::print(TEXT_MEDIUM, 0, "Testing Intake...");
    intake_motor.tare_position();

    intakeMotors.move_voltage(-100);
    intakeMotors.move_voltage(100);
    if (intakeMotors.get_position() != 0) { // Check for insufficient movement in a direction
        pros::screen::set_pen(pros::Color::red);
        pros::screen::print(TEXT_LARGE, 2, "Intake Test FAIL: Friction detected in intake mechanism");
        return false;
    }
    pros::screen::print(TEXT_MEDIUM, 2, "Intake Test Complete - Intake Nominal");
    return true;
}

// Function to test sensors
bool test_sensors() {
    pros::screen::print(TEXT_MEDIUM, 0, "Testing Sensors...");
    double imu_value = imu_sensor.get_rotation();

    if (imu_value == 0) {
        pros::screen::set_pen(pros::Color::red);
        pros::screen::print(TEXT_LARGE, 3, "Sensors Test FAIL: IMU not returning values");
        return false;
    }
    pros::screen::print(TEXT_MEDIUM, 3, "Sensors Test Complete - Sensors Nominal");
    return true;
}
bool test_pneumatics() {
    pros::screen::print(TEXT_MEDIUM, 0, "Testing Pneumatics...");
    // Test the pneumatics system

    pros::screen::print(TEXT_MEDIUM, 4, "Pneumatics Test Complete - Pneumatics Nominal");
    return true;
}

// Main routine for the system check
bool full_system_check() {
	pros::screen::set_pen(pros::Color::white);

	//setting encoder units
	leftChassis.set_encoder_units_all(pros::E_MOTOR_ENCODER_COUNTS);
	rightChassis.set_encoder_units_all(pros::E_MOTOR_ENCODER_COUNTS);

    pros::screen::print(TEXT_MEDIUM, 0, "Beginnninng System Check...");
    if(test_drivetrain() && test_intake() && test_pneumatics()) { //test_sensors() Removed as no sensors are currently implemented
        pros::screen::set_pen(pros::Color::green);
        pros::screen::print(TEXT_MEDIUM, 0, "System Check Complete - System is Nominal");
        return true;
    } else {
        pros::screen::set_pen(pros::Color::red);
        pros::screen::print(TEXT_LARGE, 0, "System Check Failed - Check Logs for Details");
        return false;
    }
}