#pragma once
#include "globals.hpp"
#include <cmath>

// ----------------------------------------------------------------------------
// Pose representation
// ----------------------------------------------------------------------------
struct Pose2D {
    double x = 0.0;        // Inches
    double y = 0.0;        // Inches
    double thetaDeg = 0.0; // Degrees, normalized to [-180, 180]
};

// ----------------------------------------------------------------------------
// Odometry: integrates encoder deltas and IMU heading to maintain pose
// Uses two perpendicular tracking wheels for accurate 2D position tracking
// ----------------------------------------------------------------------------
class Odometry {
public:
    /**
     * Construct odometry with two tracking wheels.
     * @param imuRef Reference to IMU sensor providing heading in degrees
     * @param verticalSensorRef Reference to vertical tracking wheel rotation sensor
     * @param horizontalSensorRef Reference to horizontal tracking wheel rotation sensor
     * @param wheelBaseIn Wheelbase in inches (distance between left/right wheels)
     * @param distPerTickIn Distance traveled per encoder tick (inches/tick)
     */
    Odometry(pros::IMU& imuRef, pros::Rotation& verticalSensorRef, 
             pros::Rotation& horizontalSensorRef, double wheelBaseIn, double distPerTickIn);

    // Reset pose
    void reset(double x = 0.0, double y = 0.0, double thetaDeg = 0.0);

    // Update pose from tracking wheels and IMU
    void update(double leftTicks, double rightTicks);

    // Accessors
    Pose2D pose() const { return current_; }
    double headingDeg() const { return current_.thetaDeg; }

private:
    pros::IMU& imu_;
    pros::Rotation& verticalSensor_;
    pros::Rotation& horizontalSensor_;
    double wheelBase_;
    double distPerTick_;

    double lastVerticalDist_ = 0.0;
    double lastHorizontalDist_ = 0.0;

    Pose2D current_;

    static inline double degToRad(double deg) { return deg * (M_PI / 180.0); }
    static inline double normalizeDeg(double a) {
        while (a > 180.0) a -= 360.0;
        while (a < -180.0) a += 360.0;
        return a;
    }
};
