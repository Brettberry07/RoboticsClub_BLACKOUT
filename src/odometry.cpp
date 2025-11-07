#include "odometry.hpp"

/*
================================================================================
ODOMETRY - PSEUDOCODE
================================================================================
Purpose: Track robot's position (x, y) and heading (theta) on the field
Sensors: Rotation sensor (2" tracking wheel) for distance, IMU for heading
Method: Dead reckoning using tracking wheel distance and IMU heading

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
    reset rotation sensor position to 0
    // After this, odometry tracking starts from the new position


POSITION TRACKING (MAIN LOOP)
------------------------------

void update(leftTicks, rightTicks):
    // Call this repeatedly (e.g., every 10ms) to track robot movement
    // Note: leftTicks and rightTicks parameters ignored (kept for compatibility)
    
    // Step 1: Get distance from rotation sensor
    read sensor position in centidegrees
    convert to degrees (divide by 100)
    calculate totalDistance = (degrees / 360) × trackingWheelCircumference
    
    // Step 2: Calculate distance traveled since last update
    calculate distAvg = totalDistance - lastDistance
    store totalDistance as lastDistance
    
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
- Uses dedicated 2" tracking wheel for accurate distance measurement
- IMU provides absolute heading (no drift from wheel slip)
- Call update() at high rate (100Hz) for best accuracy
- Position drift can accumulate - reset to known positions periodically

Example Usage:
    getRobot().odometry.reset(6.0, 6.0, 0.0);  // Start at (6", 6")
    
    // In loop (leftTicks/rightTicks ignored but kept for compatibility):
    getRobot().odometry.update(0, 0);
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
    
    // Reset rotation sensor to zero for new tracking
    rotationSensor.reset_position();
}

void Odometry::update(double leftTicks, double rightTicks) {
    // Use rotation sensor for linear distance and IMU for heading
    // leftTicks and rightTicks are ignored - kept for compatibility
    
    // Get distance from rotation sensor (centidegrees)
    int32_t sensorCentidegrees = rotationSensor.get_position();
    double sensorAngle = sensorCentidegrees / 100.0;  // Convert to degrees
    
    // Calculate total distance traveled from start
    double totalDistance = (sensorAngle / 360.0) * trackingWheelCircumference;
    
    // Calculate distance traveled since last update
    // Note: This static is reset by calling reset() which resets the sensor
    static double lastDistance = 0.0;
    
    // If sensor was just reset (totalDistance near 0 and lastDistance not near 0), reset tracking
    if (fabs(totalDistance) < 0.1 && fabs(lastDistance) > 0.1) {
        lastDistance = 0.0;
    }
    
    double distAvg = totalDistance - lastDistance;
    lastDistance = totalDistance;

    // Get heading from IMU
    current_.thetaDeg = normalizeDeg(imu_.get_heading());
    double headingRad = degToRad(current_.thetaDeg);

    // Update position using heading and distance
    current_.x += distAvg * std::cos(headingRad);
    current_.y += distAvg * std::sin(headingRad);

    // Mirror into existing globals for backward compatibility
    globalHeading = current_.thetaDeg;
    globalPos[0] = current_.x;
    globalPos[1] = current_.y;
}
