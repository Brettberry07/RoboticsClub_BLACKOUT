#include "odometry.hpp"

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
