#pragma once

// Test functions for verifying the two-wheel odometry system
// Add these to your test file or autonomous routines

/**
 * Test vertical tracking wheel sensor
 * Push robot forward by hand and verify reading
 */
void testVerticalSensor() {
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Testing Vertical Sensor");
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Push robot forward 12 inches...");
    
    verticalSensor.reset_position();
    pros::delay(3000);  // 3 seconds to push robot
    
    int32_t pos = verticalSensor.get_position();
    double angle = pos / 100.0;
    double distance = (angle / 360.0) * trackingWheelCircumference;
    
    pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Vertical: %.2f inches", distance);
    pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Expected: ~12.0 inches");
    
    if (distance < 0) {
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "WARNING: Negative! Set reversed=true");
    } else if (distance > 10.0 && distance < 14.0) {
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "PASS: Vertical sensor OK");
    } else {
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "CHECK: Distance seems off");
    }
}

/**
 * Test horizontal tracking wheel sensor
 * Push robot sideways by hand and verify reading
 */
void testHorizontalSensor() {
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Testing Horizontal Sensor");
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Push robot RIGHT 12 inches...");
    
    horizontalSensor.reset_position();
    pros::delay(3000);  // 3 seconds to push robot
    
    int32_t pos = horizontalSensor.get_position();
    double angle = pos / 100.0;
    double distance = (angle / 360.0) * trackingWheelCircumference;
    
    pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Horizontal: %.2f inches", distance);
    pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Expected: ~12.0 inches");
    
    if (distance < -14.0 || distance < -10.0) {
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Reading negative (pushed left)");
    } else if (distance > 10.0 && distance < 14.0) {
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "PASS: Horizontal sensor OK");
    } else {
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "CHECK: Distance seems off");
    }
}

/**
 * Test odometry accuracy with a square path
 * Robot should return to starting position
 */
void testOdometrySquare() {
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Odometry Square Test");
    
    // Start at origin facing 0 degrees
    getRobot().odometry.reset(0, 0, 0);
    
    for (int i = 0; i < 4; i++) {
        // Drive forward 24 inches
        linearPID(24.0);
        pros::delay(500);
        
        Pose2D pose = getRobot().odometry.pose();
        pros::screen::print(pros::E_TEXT_MEDIUM, 1 + i, 
                          "Side %d: (%.1f, %.1f, %.1f)", 
                          i + 1, pose.x, pose.y, pose.thetaDeg);
        
        // Turn 90 degrees right
        angularPID(imuSensor.get_heading() + 90.0);
        pros::delay(500);
    }
    
    // Check final position (should be close to origin)
    Pose2D finalPose = getRobot().odometry.pose();
    double error = sqrt(finalPose.x * finalPose.x + finalPose.y * finalPose.y);
    
    pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Final: (%.1f, %.1f)", 
                       finalPose.x, finalPose.y);
    pros::screen::print(pros::E_TEXT_MEDIUM, 6, "Position error: %.2f inches", error);
    
    if (error < 3.0) {
        pros::screen::print(pros::E_TEXT_MEDIUM, 7, "PASS: Odometry accurate!");
    } else if (error < 6.0) {
        pros::screen::print(pros::E_TEXT_MEDIUM, 7, "OK: Slight drift detected");
    } else {
        pros::screen::print(pros::E_TEXT_MEDIUM, 7, "FAIL: Large error - check sensors");
    }
}

/**
 * Continuous monitoring of both sensors
 * Useful for real-time debugging
 */
void monitorSensors() {
    while (true) {
        // Vertical sensor
        int32_t vertPos = verticalSensor.get_position();
        double vertAngle = vertPos / 100.0;
        double vertDist = (vertAngle / 360.0) * trackingWheelCircumference;
        
        // Horizontal sensor
        int32_t horizPos = horizontalSensor.get_position();
        double horizAngle = horizPos / 100.0;
        double horizDist = (horizAngle / 360.0) * trackingWheelCircumference;
        
        // IMU
        double heading = imuSensor.get_heading();
        
        // Current odometry
        Pose2D pose = getRobot().odometry.pose();
        
        pros::screen::print(pros::E_TEXT_MEDIUM, 0, "=== Sensor Monitor ===");
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Vertical:   %.2f in", vertDist);
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Horizontal: %.2f in", horizDist);
        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "IMU:        %.1f deg", heading);
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "---");
        pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Odom X:     %.2f in", pose.x);
        pros::screen::print(pros::E_TEXT_MEDIUM, 6, "Odom Y:     %.2f in", pose.y);
        pros::screen::print(pros::E_TEXT_MEDIUM, 7, "Odom Theta: %.1f deg", pose.thetaDeg);
        
        pros::delay(100);  // Update at 10Hz
        
        // Exit on controller button press
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            break;
        }
    }
}

/**
 * Test pure pursuit with updated odometry
 * Should now follow path accurately without overshooting
 */
void testPurePursuitFixed() {
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Pure Pursuit Test (Fixed)");
    
    Path testPath;
    testPath.loadFromJSON();
    
    // Get expected end position
    Point expectedEnd = testPath.curves[testPath.curveCount - 1].end;
    
    // Follow the path
    followPath(testPath);
    
    // Check actual end position
    Pose2D actualEnd = getRobot().odometry.pose();
    double errorX = actualEnd.x - expectedEnd.x;
    double errorY = actualEnd.y - expectedEnd.y;
    double totalError = sqrt(errorX * errorX + errorY * errorY);
    
    pros::screen::print(pros::E_TEXT_MEDIUM, 6, "Expected: (%.1f, %.1f)", 
                       expectedEnd.x, expectedEnd.y);
    pros::screen::print(pros::E_TEXT_MEDIUM, 7, "Actual: (%.1f, %.1f)", 
                       actualEnd.x, actualEnd.y);
    pros::screen::print(pros::E_TEXT_MEDIUM, 8, "Error: %.2f inches", totalError);
    
    if (totalError < 3.0) {
        pros::screen::print(pros::E_TEXT_MEDIUM, 9, "EXCELLENT: Path tracking fixed!");
    } else if (totalError < 6.0) {
        pros::screen::print(pros::E_TEXT_MEDIUM, 9, "GOOD: Minor tuning needed");
    } else {
        pros::screen::print(pros::E_TEXT_MEDIUM, 9, "NEEDS WORK: Check sensor setup");
    }
}

/**
 * Quick sensor health check
 * Run this at start of autonomous to verify sensors are working
 */
bool checkSensorHealth() {
    bool allGood = true;
    
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Sensor Health Check...");
    
    // Check vertical sensor
    int32_t vertPos = verticalSensor.get_position();
    if (vertPos == PROS_ERR) {
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "ERROR: Vertical sensor offline!");
        allGood = false;
    } else {
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "OK: Vertical sensor");
    }
    
    // Check horizontal sensor
    int32_t horizPos = horizontalSensor.get_position();
    if (horizPos == PROS_ERR) {
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "ERROR: Horizontal sensor offline!");
        allGood = false;
    } else {
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "OK: Horizontal sensor");
    }
    
    // Check IMU
    double heading = imuSensor.get_heading();
    if (heading == PROS_ERR_F) {
        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "ERROR: IMU offline!");
        allGood = false;
    } else {
        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "OK: IMU");
    }
    
    if (allGood) {
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "All sensors ready!");
    } else {
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "WARNING: Sensor issues detected");
    }
    
    pros::delay(2000);
    return allGood;
}

/**
 * Diagnose gear ratio - manually rotate wheel to measure sensor gearing
 * This helps determine the correct trackingWheelGearRatio value
 */
void diagnoseGearRatio() {
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "=== Gear Ratio Diagnosis ===");
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "");
    pros::screen::print(pros::E_TEXT_MEDIUM, 2, "VERTICAL WHEEL:");
    pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Rotate wheel 1 full turn...");
    
    verticalSensor.reset_position();
    pros::delay(5000);  // 5 seconds to rotate
    
    int32_t vertPos = verticalSensor.get_position();
    double vertRotations = vertPos / 36000.0;  // centidegrees to rotations
    
    pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Sensor: %.3f rotations", vertRotations);
    
    if (vertRotations > 2.5 && vertRotations < 3.5) {
        pros::screen::print(pros::E_TEXT_MEDIUM, 5, "LIKELY: 3:1 ratio, use 0.333");
    } else if (vertRotations > 0.9 && vertRotations < 1.1) {
        pros::screen::print(pros::E_TEXT_MEDIUM, 5, "LIKELY: 1:1 direct, use 1.0");
    } else if (vertRotations > 0.25 && vertRotations < 0.4) {
        pros::screen::print(pros::E_TEXT_MEDIUM, 5, "LIKELY: 1:3 ratio, use 3.0");
    } else {
        double calculatedRatio = 1.0 / vertRotations;
        pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Custom ratio: %.3f", calculatedRatio);
    }
    
    pros::delay(3000);
    
    // Test horizontal
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "");
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "HORIZONTAL WHEEL:");
    pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Rotate wheel 1 full turn...");
    
    horizontalSensor.reset_position();
    pros::delay(5000);
    
    int32_t horizPos = horizontalSensor.get_position();
    double horizRotations = horizPos / 36000.0;
    
    pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Sensor: %.3f rotations", horizRotations);
    
    if (horizRotations > 2.5 && horizRotations < 3.5) {
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "LIKELY: 3:1 ratio, use 0.333");
    } else if (horizRotations > 0.9 && horizRotations < 1.1) {
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "LIKELY: 1:1 direct, use 1.0");
    } else if (horizRotations > 0.25 && horizRotations < 0.4) {
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "LIKELY: 1:3 ratio, use 3.0");
    } else {
        double calculatedRatio = 1.0 / horizRotations;
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Custom ratio: %.3f", calculatedRatio);
    }
    
    pros::delay(3000);
    pros::screen::print(pros::E_TEXT_MEDIUM, 5, "");
    pros::screen::print(pros::E_TEXT_MEDIUM, 6, "Update trackingWheelGearRatio");
    pros::screen::print(pros::E_TEXT_MEDIUM, 7, "in src/globals.cpp");
}

/**
 * Calibration test - drive known distance and measure actual travel
 */
void calibrateGearRatio() {
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "=== Gear Ratio Calibration ===");
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Place robot on field");
    pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Mark starting position");
    pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Press A to start...");
    
    while (!master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
        pros::delay(20);
    }
    
    getRobot().odometry.reset(0, 0, 0);
    
    pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Driving 40 inches...");
    linearPID(40.0);
    
    Pose2D pose = getRobot().odometry.pose();
    
    pros::screen::print(pros::E_TEXT_MEDIUM, 4, "");
    pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Commanded: 40.0 inches");
    pros::screen::print(pros::E_TEXT_MEDIUM, 6, "Odometry:  %.2f inches", pose.x);
    pros::screen::print(pros::E_TEXT_MEDIUM, 7, "Measure with tape!");
    pros::screen::print(pros::E_TEXT_MEDIUM, 8, "Ratio = 40 / measured");
    
    // Wait for user to measure
    pros::delay(10000);
}
