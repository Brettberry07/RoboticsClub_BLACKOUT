#include "odometry.hpp"

/*
================================================================================
ODOMETRY - PSEUDOCODE (TWO-WHEEL SYSTEM)
================================================================================
Purpose: Track robot's position (x, y) and heading (theta) on the field
Sensors: Two perpendicular tracking wheels (vertical & horizontal) + IMU
Method: Dead reckoning using two tracking wheels for 2D motion and IMU for heading

INITIALIZATION
--------------

Odometry(imuRef, verticalSensorRef, horizontalSensorRef, wheelBaseIn, distPerTickIn):
    // Constructor - set up odometry with sensors and calibration
    store reference to IMU sensor
    store reference to vertical tracking wheel sensor
    store reference to horizontal tracking wheel sensor
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
    reset vertical sensor position to 0
    reset horizontal sensor position to 0
    reset lastVerticalDist to 0
    reset lastHorizontalDist to 0
    // After this, odometry tracking starts from the new position


POSITION TRACKING (MAIN LOOP) - TWO-WHEEL ODOMETRY
--------------------------------------------------

void update(leftTicks, rightTicks):
    // Call this repeatedly (e.g., every 10ms) to track robot movement
    // Note: leftTicks and rightTicks parameters ignored (kept for compatibility)
    
    // Step 1: Get distance from vertical tracking wheel (forward/backward motion)
    read vertical sensor position in centidegrees
    convert to degrees (divide by 100)
    calculate verticalDist = (degrees / 360) × trackingWheelCircumference
    
    // Step 2: Get distance from horizontal tracking wheel (left/right motion)
    read horizontal sensor position in centidegrees
    convert to degrees (divide by 100)
    calculate horizontalDist = (degrees / 360) × trackingWheelCircumference
    
    // Step 3: Calculate distance traveled since last update (local frame)
    calculate deltaVertical = verticalDist - lastVerticalDist
    calculate deltaHorizontal = horizontalDist - lastHorizontalDist
    store verticalDist as lastVerticalDist
    store horizontalDist as lastHorizontalDist
    
    // Step 4: Get current heading from IMU
    read IMU heading and normalize to [-180, 180] degrees
    store as current thetaDeg
    convert current thetaDeg to radians for math calculations
    
    // Step 5: Transform local movement to global coordinate frame
    // Vertical wheel measures forward motion (along robot's X-axis)
    // Horizontal wheel measures lateral motion (along robot's Y-axis)
    // Rotate these movements by robot heading to get global X and Y changes
    calculate deltaX_global = deltaVertical × cos(heading) - deltaHorizontal × sin(heading)
    calculate deltaY_global = deltaVertical × sin(heading) + deltaHorizontal × cos(heading)
    
    // Step 6: Update global position
    add deltaX_global to current x position
    add deltaY_global to current y position
    
    // Step 7: Mirror to legacy global variables for backward compatibility
    copy current thetaDeg to globalHeading
    copy current x to globalPos[0]
    copy current y to globalPos[1]


================================================================================
NOTES
================================================================================
- Uses TWO perpendicular 2" tracking wheels for accurate 2D position tracking
- Vertical wheel: measures forward/backward motion
- Horizontal wheel: measures left/right (strafe) motion
- IMU provides absolute heading (no drift from wheel slip)
- This system accounts for all motion including sideways drift during turns
- Call update() at high rate (100Hz) for best accuracy
- Position drift can accumulate - reset to known positions periodically

Example Usage:
    getRobot().odometry.reset(6.0, 6.0, 0.0);  // Start at (6", 6")
    
    // In loop (leftTicks/rightTicks ignored but kept for compatibility):
    getRobot().odometry.update(0, 0);
    Pose2D pose = getRobot().odometry.pose();

================================================================================
*/

Odometry::Odometry(pros::IMU& imuRef, pros::Rotation& verticalSensorRef,
                   pros::Rotation& horizontalSensorRef, double wheelBaseIn, double distPerTickIn)
    : imu_(imuRef), verticalSensor_(verticalSensorRef), horizontalSensor_(horizontalSensorRef),
      wheelBase_(wheelBaseIn), distPerTick_(distPerTickIn) {
    reset(0.0, 0.0, 0.0);
}

void Odometry::reset(double x, double y, double thetaDeg) {
    current_.x = x;
    current_.y = y;
    current_.thetaDeg = normalizeDeg(thetaDeg);
    lastVerticalDist_ = 0.0;
    lastHorizontalDist_ = 0.0;
    
    // Reset both tracking wheels to zero for new tracking
    verticalSensor_.reset_position();
    horizontalSensor_.reset_position();
}

void Odometry::update(double leftTicks, double rightTicks) {
    // Use two perpendicular tracking wheels for 2D motion and IMU for heading
    // leftTicks and rightTicks are ignored - kept for compatibility
    
    // Get distance from vertical tracking wheel (forward/backward motion)
    int32_t verticalCentidegrees = verticalSensor_.get_position();
    double verticalAngle = verticalCentidegrees / 100.0;  // Convert to degrees
    double verticalDist = (verticalAngle / 360.0) * trackingWheelCircumference;
    
    // Get distance from horizontal tracking wheel (left/right motion)
    int32_t horizontalCentidegrees = horizontalSensor_.get_position();
    double horizontalAngle = horizontalCentidegrees / 100.0;  // Convert to degrees
    double horizontalDist = (horizontalAngle / 360.0) * trackingWheelCircumference;
    
    // Calculate change in distance since last update (robot's local frame)
    double deltaVertical = verticalDist - lastVerticalDist_;
    double deltaHorizontal = horizontalDist - lastHorizontalDist_;
    lastVerticalDist_ = verticalDist;
    lastHorizontalDist_ = horizontalDist;

    // Get heading from IMU
    current_.thetaDeg = normalizeDeg(imu_.get_heading());
    double headingRad = degToRad(current_.thetaDeg);

    // Transform local movement (robot frame) to global coordinate frame
    // Vertical wheel measures forward (robot's local X), horizontal measures strafe (robot's local Y)
    double deltaX_global = deltaVertical * std::cos(headingRad) - deltaHorizontal * std::sin(headingRad);
    double deltaY_global = deltaVertical * std::sin(headingRad) + deltaHorizontal * std::cos(headingRad);
    
    // Update global position
    current_.x += deltaX_global;
    current_.y += deltaY_global;

    // Mirror into existing globals for backward compatibility
    globalHeading = current_.thetaDeg;
    globalPos[0] = current_.x;
    globalPos[1] = current_.y;
}
