#include "globals.hpp"
#include <numeric>
#include "pros/apix.h"
#include "pros/rtos.hpp"


// Function to test drivetrain
bool test_drivetrain() {
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Testing Drivetrain...                                                             ");
    driveTrainMotors.tare_position();

    driveTrainMotors.move_voltage(-1200); // Move drivetrain backward for 1 second
    pros::delay(50);

    driveTrainMotors.move_voltage(1200);// Move drivetrain forward for 1 second
    pros::delay(50); 

    if (driveTrainMotors.get_position() > 1 || driveTrainMotors.get_position() < 1) { // Check for insufficient movement in a direction
        pros::screen::set_pen(pros::Color::red);
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Drivetrain Test FAIL: Motors firing at inconsistent rates, check temps                                                                                           ");

        std::vector<double> leftTemperatures = leftChassis.get_temperature_all();
        std::vector<double> rightTemperatures = leftChassis.get_temperature_all();
        
        double leftTemp = std::accumulate(leftTemperatures.begin(), leftTemperatures.end(), 0.0) / leftTemperatures.size();
        double rightTemp = std::accumulate(rightTemperatures.begin(), rightTemperatures.end(), 0.0) / rightTemperatures.size();

        // Print the average temperatures of the motors and return false
        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Left Chassis temp average: %f, Right: %f                                                                                                                         ", leftTemp, rightTemp);
        pros::delay(500);
        return false;
    }

    pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Drivetrain Test PASS - Drivetrain nominal                                                                                           ");
    return true;
}

// Function to test intake
bool test_intake() {
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Testing Intake...                                                                                           ");
    intakeMotors.tare_position();

    intakeMotors.move_voltage(-1000);
    intakeMotors.move_voltage(1000);
    if (intakeMotors.get_position() != 0) { // Check for insufficient movement in a direction
        pros::screen::set_pen(pros::Color::red);
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Intake Test FAIL: Friction detected in intake mechanism                                                                                           ");
        return false;
    }
    pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Intake Test Complete - Intake Nominal                                                                                           ");
    return true;
}

// Function to test sensors
bool test_sensors() {
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Testing Sensors...                                                                                           ");
    try { 
        double imu_value = imuSensor.get_rotation();
        if (imu_value != 0) {
            throw new std::exception();
        }
    } catch(const std::exception& error) {
        pros::screen::set_pen(pros::Color::red);
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Sensors Test FAIL: IMU not calibrated                                                                                          ");
        return false;
    }

    pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Sensors Test Complete - Sensors Nominal                                                            ");
    return true;
}

// Test the pneumatics system
bool test_pneumatics() {
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Testing Pneumatics... - !!!Watch CLAMP!!!                                                                                           ");
    
    for (int i = 0; i < 4; i++)
    {
        setAutonPin(HIGH, clampPin);
        pros::delay(300);
        setAutonPin(LOW, clampPin);
        pros::delay(300);
    }
    pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Pneumatics Test Complete                                                             ");
    return true;
}

// Function to test collisions
bool test_collisions() {
    driveTrainMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    leftChassis.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    rightChassis.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);


    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Testing Collisions...                                                                                           ");

    for (int i = 0; i < 5; i++) {
        // Test Head-on collisions
        driveTrainMotors.move_voltage(-12000); 
        pros::delay(200);
        driveTrainMotors.brake();
        pros::delay(300); // Delay for 0.1 seconds
        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Head-on collision test, Number: %d                                                                                           ", i);


        // Test Backward collisions
        driveTrainMotors.move_voltage(12000); // Move drivetrain forward quickly
        pros::delay(200);
        driveTrainMotors.brake();
        pros::delay(300); // Delay for 0.5 seconds
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Backwards collision test, Number: %d                                                                                                                         ", i);


        // Test Left Side collisions
        leftChassis.move_voltage(12000); // Move drivetrain to the left
        rightChassis.move_voltage(-12000); // Move right chassis to the left
        pros::delay(200);
        leftChassis.move_voltage(-12000);
        rightChassis.move_voltage(12000);
        pros::delay(200);
        driveTrainMotors.brake();   
        pros::delay(300); // Delay for 0.1 seconds

        // Test Right Side collisions
        rightChassis.move_voltage(12000); // Move drivetrain to the right
        leftChassis.move_voltage(-12000); // Move left chassis to the right
        pros::delay(200);
        rightChassis.move_voltage(-12000);
        leftChassis.move_voltage(12000);
        pros::delay(200); // Delay for 0.2 seconds
        driveTrainMotors.brake();
        pros::delay(300); // Delay for 0.1 seconds
    }

    driveTrainMotors.move_voltage(0); // Ensure Drivetrain is stopped
    pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Collisions Test Complete - Bot condition Nominal                                                                                           ");
    return true;
}

// Main routine for the system check
/**
 * @brief A full self-verifying systems check.
 * 
 * This function checks the various components of the system, including the drivetrain, intake, sensors and pneumatics.
 * If all components pass the tests, the system check is considered complete and the function returns true.
 * Otherwise, if any component fails the tests, the system check is considered failed and the function returns false.
 * 
 * @return True if the system check is complete and all components pass the tests, False otherwise.
 */
bool full_system_check() {
	pros::screen::set_pen(pros::Color::white);

    driveTrainMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Beginnninng System Check...                                                             ");
    if(test_drivetrain() && test_intake() && test_collisions() && test_pneumatics() && test_sensors()) {
        pros::screen::set_pen(pros::Color::green);
        pros::screen::print(pros::E_TEXT_MEDIUM, 0, "System Check Complete - System is Nominal                                                             ");
        driveTrainMotors.brake();
        return true;
    } else {
        pros::screen::set_pen(pros::Color::red);
        pros::screen::print(pros::E_TEXT_MEDIUM, 0, "System Check Failed - Check Logs for Details                                                             ");
        driveTrainMotors.brake();
        return false;
    }
}