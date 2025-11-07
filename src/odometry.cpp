#include "odometry.hpp"

/*
================================================================================
ODOMETRY - PSEUDOCODE
================================================================================
Purpose: Track robot's position (x, y) and heading (theta) on the field
Sensors: IMU for absolute heading, motor encoders for distance traveled
Method: Differential drive odometry with IMU integration

INITIALIZATION
--------------

Odometry(imuRef, wheelBaseIn, distPerTickIn):
    // Constructor - set up odometry with sensors and calibration
    store reference to IMU sensor
    store wheelBase (distance between left/right wheels in inches)
    store distPerTick (inches traveled per encoder tick)
    call reset(0, 0, 0) to initialize position at origin


POSITION MANAGEMENT
-------------------

void reset(x, y, thetaDeg):
    // Set robot to known position and heading (used at autonomous start)
    set current x position = x
    set current y position = y
    normalize thetaDeg to [-180, 180] and store as current heading
    reset lastLeftTicks to 0
    reset lastRightTicks to 0
    // After this, odometry tracking starts from the new position


POSITION TRACKING (MAIN LOOP)
------------------------------

void update(leftTicks, rightTicks):
    // Call this repeatedly (e.g., every 10ms) to track robot movement
    
    // Step 1: Calculate distance traveled since last update
    calculate dLeft = leftTicks - lastLeftTicks
    calculate dRight = rightTicks - lastRightTicks
    store leftTicks as lastLeftTicks
    store rightTicks as lastRightTicks
    
    // Step 2: Convert encoder ticks to actual distance in inches
    calculate distLeft = dLeft × distPerTick
    calculate distRight = dRight × distPerTick
    calculate distAvg = (distLeft + distRight) / 2
    // distAvg is the straight-line distance the robot's center moved
    
    // Step 3: Get current heading from IMU
    read IMU heading and normalize to [-180, 180] degrees
    store as current thetaDeg
    convert current thetaDeg to radians for math calculations
    
    // Step 4: Update position using heading and distance traveled
    add distAvg × cos(heading) to current x position
    add distAvg × sin(heading) to current y position
    
    // Step 5: Mirror to legacy global variables for backward compatibility
    copy current thetaDeg to globalHeading
    copy current x to globalPos[0]
    copy current y to globalPos[1]


================================================================================
NOTES
================================================================================
- Differential drive odometry uses encoder deltas to estimate distance
- IMU provides absolute heading (corrects for wheel slip in turns)
- Call update() at high rate (100Hz) for best accuracy
- Position drift accumulates over time - reset to known positions periodically

Example Usage:
    getRobot().odometry.reset(6.0, 6.0, 0.0);  // Start at (6", 6")
    
    // In loop:
    getRobot().odometry.update(leftEncoder, rightEncoder);
    Pose2D pose = getRobot().odometry.pose();

================================================================================
*/

Odometry::Odometry(pros::IMU& imuRef, double wheelBaseIn, double distPerTickIn)
    : imu_(imuRef), wheelBase_(wheelBaseIn), distPerTick_(distPerTickIn) {
    reset(0.0, 0.0, 0.0);
}

void Odometry::reset(double x, double y, double thetaDeg) {
    current_.x = x;
    current_.y = y;
    current_.thetaDeg = normalizeDeg(thetaDeg);
    lastLeftTicks_ = 0.0;
    lastRightTicks_ = 0.0;
}

void Odometry::update(double leftTicks, double rightTicks) {
    // Integrate using delta ticks and IMU heading (degrees)
    double dLeft = leftTicks - lastLeftTicks_;
    double dRight = rightTicks - lastRightTicks_;
    lastLeftTicks_ = leftTicks;
    lastRightTicks_ = rightTicks;

    double distLeft = dLeft * distPerTick_;
    double distRight = dRight * distPerTick_;
    double distAvg = (distLeft + distRight) / 2.0;

    current_.thetaDeg = normalizeDeg(imu_.get_heading());
    double headingRad = degToRad(current_.thetaDeg);

    current_.x += distAvg * std::cos(headingRad);
    current_.y += distAvg * std::sin(headingRad);

    // Mirror into existing globals for backward compatibility
    globalHeading = current_.thetaDeg;
    globalPos[0] = current_.x;
    globalPos[1] = current_.y;
}
