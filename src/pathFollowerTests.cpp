#include "pathFollower.hpp"
#include "globals.hpp"
#include "robot.hpp"

/**
 * Example usage of the path following system.
 */

void testPathFollower() {
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Starting path follower test");
    
    // Create and load the path from JSON.
    Path testPath;
    testPath.loadFromJSON();

    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "path starting point: (%.1f, %.1f)", testPath.curves[0].start.x, testPath.curves[0].start.y);

    // Follow the complete path.
    followPath(testPath);
    
    pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Test complete!");
}

void testMoveTo() {
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Testing moveTo function");
    
    // Example: Move to a point 24 inches forward and 12 inches to the right.
    Point targetPoint(24.0, 12.0);
    
    // Move to the point (without specific heading requirement).
    moveTo(targetPoint);
    
    pros::screen::print(pros::E_TEXT_MEDIUM, 5, "MoveTo complete!");
}

void testMoveToWithHeading() {
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Testing moveTo with heading");
    
    // Example: Move to a point and face 90 degrees.
    Point targetPoint(24.0, 12.0);
    double targetHeading = 90.0;  // Face 90 degrees at end
    
    moveTo(targetPoint, targetHeading);
    
    pros::screen::print(pros::E_TEXT_MEDIUM, 5, "MoveTo with heading complete!");
}

void testIndividualCurve() {
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Testing single curve");
    
    // Create a path with just the first curve.
    Path singleCurvePath;
    
    BezierCurve curve1;
    curve1.curve = 1;
    curve1.start = Point(116.13, 123.5);
    curve1.control1 = Point(122, 241);
    curve1.middle = Point(180.77, 240.69);
    curve1.control2 = Point(241, 240);
    curve1.end = Point(241, 359);
    curve1.reversed = false;
    curve1.convertToInches();
    
    singleCurvePath.curves[singleCurvePath.curveCount++] = curve1;
    
    // Follow just this one curve.
    followPath(singleCurvePath);
    
    pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Single curve test complete!");
}

void tunePathFollower() {
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Tuning path follower");
    
    // Create a simple test path.
    Path testPath;
    testPath.loadFromJSON();
    testPath.generateWaypoints(50);
    
    // Create controller with custom settings.
    PurePursuitController controller;
    controller.setPath(&testPath);
    
    // Try different configurations.
    PurePursuitConfig config;
    
    // Configuration 1: Conservative.
    config.lookaheadDistance = 8.0;   // Shorter lookahead = tighter following
    config.maxSpeed = 60.0;            // Slower for testing
    config.minSpeed = 15.0;
    config.turnDamping = 0.7;          // More damping = slower in turns
    config.positionTolerance = 2.0;
    
    controller.setConfig(config);
    
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Config: LA=%.1f, Max=%.0f", 
                       config.lookaheadDistance, config.maxSpeed);
    
    // Run the path.
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
    
    pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Tuning test complete!");
}
