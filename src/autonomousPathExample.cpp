#include "globals.hpp"
#include "pathFollower.hpp"
#include "pathFollowerTests.hpp"
#include "robot.hpp"

/**
 * @file autonomousPathExample.cpp
 * @brief Example autonomous routines using the path follower.
 *
 * Copy any of these functions into your autonomous.cpp or call them from autonomous().
 */

/**
 * @brief Simple autonomous — follow the complete JSON path.
 */
void autonomousFollowPath() {
    // 1. Initialize
    imuSensor.reset();
    getRobot().drivetrain.tare();
    getRobot().odometry.reset(0.0, 0.0, 0.0);
    
    pros::delay(2000);  // Wait for IMU calibration.
    
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Starting Path Follower");
    
    // 2. Load and follow the path.
    Path autonomousPath;
    autonomousPath.loadFromJSON();
    followPath(autonomousPath);
    
    pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Path Complete!");
}

/**
 * @brief Autonomous with multiple moveTo commands.
 */
void autonomousMultiplePoints() {
    // Initialize.
    imuSensor.reset();
    getRobot().drivetrain.tare();
    getRobot().odometry.reset(0.0, 0.0, 0.0);
    
    pros::delay(2000);
    
    // Move to first point.
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Moving to point 1");
    moveTo(Point(24.0, 0.0));  // 24 inches forward
    
    pros::delay(500);
    
    // Move to second point.
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Moving to point 2");
    moveTo(Point(24.0, 24.0), 90.0);  // Right and face 90°
    
    pros::delay(500);
    
    // Move to third point.
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Moving to point 3");
    moveTo(Point(0.0, 24.0), 180.0);  // Left and face 180°
    
    pros::delay(500);
    
    // Return to start.
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Returning to start");
    moveTo(Point(0.0, 0.0), 0.0);
    
    pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Square Complete!");
}

/**
 * @brief Autonomous with custom tuning for precision.
 */
void autonomousPrecisionPath() {
    // Initialize.
    imuSensor.reset();
    getRobot().drivetrain.tare();
    getRobot().odometry.reset(0.0, 0.0, 0.0);
    
    pros::delay(2000);
    
    // Load path.
    Path precisionPath;
    precisionPath.loadFromJSON();
    precisionPath.generateWaypoints(75);  // More waypoints = more precision
    
    // Create controller with custom tuning.
    PurePursuitController controller;
    controller.setPath(&precisionPath);
    
    // Precision configuration.
    PurePursuitConfig config;
    config.lookaheadDistance = 8.0;   // Shorter for tighter following
    config.maxSpeed = 70.0;            // Slower for accuracy
    config.minSpeed = 20.0;
    config.turnDamping = 0.7;          // More damping in turns
    config.positionTolerance = 1.5;    // Tighter tolerance
    controller.setConfig(config);
    
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Precision Mode");
    
    // Main control loop.
    getRobot().drivetrain.tare();
    
    while (!controller.hasReachedEnd(getCurrentPosition(), getRobot().odometry.headingDeg())) {
        double leftTicks, rightTicks;
        getRobot().drivetrain.getPosition(leftTicks, rightTicks);
        updateOdom(leftTicks, rightTicks);
        
        Point currentPos = getCurrentPosition();
        double leftSpeed, rightSpeed;
        controller.calculateMotorSpeeds(currentPos, getRobot().odometry.headingDeg(), leftSpeed, rightSpeed);
        
    getRobot().drivetrain.setOpenLoop(static_cast<int>(leftSpeed), static_cast<int>(rightSpeed));
        
        pros::delay(20);
    }
    
    getRobot().drivetrain.brake();
    
    pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Precision Complete!");
}

/**
 * @brief Autonomous with fast, aggressive tuning.
 */
void autonomousFastPath() {
    // Initialize.
    imuSensor.reset();
    getRobot().drivetrain.tare();
    getRobot().odometry.reset(0.0, 0.0, 0.0);
    
    pros::delay(2000);
    
    // Load path.
    Path fastPath;
    fastPath.loadFromJSON();
    fastPath.generateWaypoints(40);  // Fewer waypoints for speed
    
    // Create controller with aggressive tuning.
    PurePursuitController controller;
    controller.setPath(&fastPath);
    
    // Fast configuration.
    PurePursuitConfig config;
    config.lookaheadDistance = 15.0;  // Longer for smoother
    config.maxSpeed = 120.0;           // Max speed!
    config.minSpeed = 30.0;
    config.turnDamping = 0.4;          // Less damping = faster turns
    config.positionTolerance = 3.0;    // Looser tolerance
    controller.setConfig(config);
    
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "FAST MODE");
    
    // Main control loop.
    getRobot().drivetrain.tare();
    
    while (!controller.hasReachedEnd(getCurrentPosition(), getRobot().odometry.headingDeg())) {
        double leftTicks, rightTicks;
        getRobot().drivetrain.getPosition(leftTicks, rightTicks);
        updateOdom(leftTicks, rightTicks);
        
        Point currentPos = getCurrentPosition();
        double leftSpeed, rightSpeed;
        controller.calculateMotorSpeeds(currentPos, getRobot().odometry.headingDeg(), leftSpeed, rightSpeed);
        
    getRobot().drivetrain.setOpenLoop(static_cast<int>(leftSpeed), static_cast<int>(rightSpeed));
        
        pros::delay(20);
    }
    
    getRobot().drivetrain.brake();
    
    pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Fast Complete!");
}

/**
 * @brief Test just the first curve from the JSON.
 */
void autonomousTestFirstCurve() {
    // Initialize.
    imuSensor.reset();
    getRobot().drivetrain.tare();
    getRobot().odometry.reset(0.0, 0.0, 0.0);
    
    pros::delay(2000);
    
    // Create path with just first curve.
    Path testPath;
    
    BezierCurve curve1;
    curve1.curve = 1;
    curve1.start = Point(116.13, 123.5);
    curve1.control1 = Point(122, 241);
    curve1.middle = Point(180.77, 240.69);
    curve1.control2 = Point(241, 240);
    curve1.end = Point(241, 359);
    curve1.reversed = false;
    curve1.convertToInches();
    
    testPath.curves[testPath.curveCount++] = curve1;
    
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Testing First Curve");
    
    followPath(testPath);
    
    pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Test Complete!");
}
