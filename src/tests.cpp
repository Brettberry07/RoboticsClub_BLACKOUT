#include "globals.hpp"
#include <numeric>
#include "pros/apix.h"
#include "pros/rtos.hpp"


/**
 * @brief Test the drivetrain functionality.
 * 
 * This function tests the drivetrain by moving it backward and forward for a short duration and checking if the motors are firing at consistent rates. 
 * If the movement is inconsistent, it prints the average temperatures of the motors and returns false. Otherwise, it prints a pass message and returns true.
 * 
 * @return True if the drivetrain test passes, false otherwise.
 */
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
/**
 * @brief Test the intake mechanism.
 * 
 * This function tests the intake mechanism by moving the intake motors in both directions and checking for any movement. 
 * If there is no movement detected, the test is considered successful. Otherwise, it is considered a failure due to friction in the intake mechanism.
 * 
 * @return True if the test is successful, false otherwise.
 */
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
/**
 * @brief Test the sensors.
 * 
 * This function tests the sensors by checking the calibration of the IMU sensor.
 * It prints the test result on the screen.
 * 
 * @return True if the sensors are calibrated, false otherwise.
 */
bool test_sensors() {
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Testing Sensors...                                                                                           ");
    try { 
        double imu_value = imuSensor.get_rotation();
        if (imu_value > 10 || imu_value < -10) {
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


/**
 * @brief Test the pneumatics functionality.
 * 
 * This function tests the pneumatics by repeatedly activating and deactivating a pneumatic clamp (total of 3 times).
 * It prints messages on the screen to indicate the progress of the test.
 * 
 * @attention This test will always return true becuase the only failure condition cannot be observed by
 * the program.
 * 
 * @return true 
 */
bool test_pneumatics() {
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Testing Pneumatics... - !!!Watch CLAMP!!!                                                                                           ");
    
    for (int i = 0; i < 3; i++)
    {
        setAutonPin(HIGH, clampPin);
        pros::delay(300);
        setAutonPin(LOW, clampPin);
        pros::delay(300);
    }
    pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Pneumatics Test Complete                                                             ");
    return true;
}

/**
 * @brief Tests rigidity of the robot.
 * 
 * This function tests different types of collisions for the robot. It simulates head-on collisions, backward collisions,
 * left side collisions, and right side collisions. The purpose of this test is to ensure that the robot can handle
 * collisions and withstand any damages. The test is repeated 5 times to gather sufficient data.
 * 
 * The function sets the brake mode for the drive train motors, left chassis, and right chassis to hold, so we can stop
 * the robot quick enough to be effective at simulating the collisions.
 * 
 * @tableofcontents 
 * For each iteration of the test, the function performs the following steps:
 * 1. Head-on collision test: The drive train motors move in forward at a high voltage to gain speed.
 *    After a short delay, the motors apply maximum brakes simulating a head-on collision and display 
 *    a message on the screen indicating the test number.
 * 2. Backward collision test: The drive train motors move backwards at a high voltage to gain speed. After a short delay, 
 *    the motors apply maximum brakes simulating a rear collision and display a message on the screen indicating 
 *    the test number.
 * 3. Left side collision test: The left chassis and right chassis move in opposite directions, gaining speed in one 
 *    rotational direction. After a short delay, the motors apply maximum brakes simulating a collision and display 
 *    a message on the screen indicating the test and number.
 * 4. Right side collision test: The right chassis and left chassis move in opposite directions, gaining speed in the 
 *    opposite direction as the other test. After a short delay, the motors apply maximum brakes simulating a collision 
 *     display a message on the screen indicating the test and number.
 * 
 * After the test is completed, the drive train motors are stopped and a message is displayed on the screen indicating
 * that the test is complete and the robot is in a nominal condition.
 * 
 * @attention Ensure that the robot is in a safe environment and that there are no obstacles in the path of the robot.
 *  The robot should be placed on a flat surface to ensure accurate results and manual supervision is required
 *  to check robot status throughout the tests.
 * 
 * @return true if the test is completed successfully, false otherwise.
 */
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

/**
 * @brief Function to test the batteries.
 * 
 * This function checks the battery levels of the controller and the robot. It prints the battery level of the controller and the robot on the screen. If the battery percentage of the robot is below 20%, it sets the screen color to red, prints a low battery warning message, and triggers a rumble on the controller. Otherwise, it prints a nominal battery level message.
 * 
 * @return true if the battery level is nominal, false if the battery level is low.
 */
bool test_batteries() {
    // Check controller battery level
    master.print(0, 0, "Battery Level: %d\n", master.get_battery_level());

    // Check the robot battery voltage (Ranges from 14.6 volts to 10 volts)
    double battery_capacity = pros::battery::get_capacity();
    double battery_percentage = (battery_capacity / 14.6) * 100; // Calculate battery percentage

    pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Robot Battery Level: %.2f%%", battery_percentage);
    if (battery_percentage < 20) {
        pros::screen::set_pen(pros::Color::red);
        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Battery Level Low - Charge Battery Immediately                                                                                           ");
        const char* tempRumble = "...- - -..."; // "-" is long, "." is short, " " is a pause
        master.rumble(tempRumble);
        return false;
    } else {
        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Battery Level Nominal                                                                                    ");
        return true;
    }
}

/**
 * @brief A full self-verifying systems check.
 * 
 * This function checks the various components of the system, including the drivetrain, intake, sensors and pneumatics. If all components pass the tests, the system check is considered complete and the function returns true. Otherwise, if any component fails the tests, the system check is considered failed and the function returns false.
 * 
 * @return True if the system check is complete and all components pass the tests, False otherwise.
 */
bool full_system_check() {
	pros::screen::set_pen(pros::Color::white);

    driveTrainMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Beginnninng System Check...");
    if(test_drivetrain() && test_intake() && test_collisions() && test_pneumatics() && test_sensors() && test_batteries()) {
        pros::screen::set_pen(pros::Color::green);
        pros::screen::print(pros::E_TEXT_MEDIUM, 0, "System Check Complete - System is Nominal");
        driveTrainMotors.brake();
        return true;
    } else {
        pros::screen::set_pen(pros::Color::red);
        pros::screen::print(pros::E_TEXT_MEDIUM, 0, "System Check Failed - System requires attention");
        driveTrainMotors.brake();
        return false;
    }
}

/**,
 * @brief a low profile check to ensure that the robot is in a safe state.
 * 
 * This function checks if the motors are at rest and if the IMU sensor is calibrated.
 * If any of these conditions fail, an error message is displayed on the screen and the function returns false.
 * 
 * @return True if the low profile check passes and the robot is in a safe state, false otherwise.
 */
bool low_profile_check() {
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Performing Low Profile Check...");

    std::vector<double> leftTemperatures = leftChassis.get_temperature_all();
    std::vector<double> rightTemperatures = leftChassis.get_temperature_all();
        
    double leftTemp = std::accumulate(leftTemperatures.begin(), leftTemperatures.end(), 0.0) / leftTemperatures.size();
    double rightTemp = std::accumulate(rightTemperatures.begin(), rightTemperatures.end(), 0.0) / rightTemperatures.size();

    double avgTemp = leftTemp + rightTemp / 2;
    if (test_sensors() && test_batteries() && avgTemp < 80) {
        master.print(0, 0, "Low Profile Check PASS - Robot is in a safe state");
        return true;
    } else {
        master.rumble("--------");
        master.print(0, 0, "Low Profile Check FAIL - Robot requires attention");
        return false;
    }
}
bool real_time_test() {
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Performing Low Profile Check...");

    std::vector<double> leftTemperatures = leftChassis.get_temperature_all();
    std::vector<double> rightTemperatures = leftChassis.get_temperature_all();
        
    double leftTemp = std::accumulate(leftTemperatures.begin(), leftTemperatures.end(), 0.0) / leftTemperatures.size();
    double rightTemp = std::accumulate(rightTemperatures.begin(), rightTemperatures.end(), 0.0) / rightTemperatures.size();
    
    double avgTemp = leftTemp + rightTemp / 2;
    
    int overTemp = driveTrainMotors.is_over_temp_all()[0]; // Check if motors are above rated temperature
   
    if (test_batteries() && overTemp != 1) {
        return true;
    } else {
        if (overTemp == 1) {
            master.rumble("--------");
            master.print(0, 0, " Check FAIL - Motor Overheating");
            master.print(1, 0 , "Motor Temp: %f", avgTemp);
        } 
        if (!test_batteries()) {
            master.rumble("........");
            master.print(0, 0, " Check FAIL - Battery is below 20%, drive conservatively");
        }
        return false;
    }
}