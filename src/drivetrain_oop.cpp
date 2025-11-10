#include "drivetrain_oop.hpp"

/*
================================================================================
DRIVETRAIN OOP - PSEUDOCODE
================================================================================

DRIVER CONTROL MODES
--------------------

void tank(leftY, rightY):
    // Classic tank drive - left stick controls left, right stick controls right
    if curved mode enabled:
        apply cubic curve to leftY
        apply cubic curve to rightY
    move left motors at leftY
    move right motors at rightY

void split(power, turn):
    // Split arcade - forward/back on one stick, turn on another
    calculate left = power + turn
    calculate right = power - turn
    if curved mode enabled:
        apply cubic curve to left
        apply cubic curve to right
    move left motors at left
    move right motors at right

void arcade(power, turn):
    // Single-stick arcade - Y for forward/back, X for turn
    calculate left = power + turn
    calculate right = power - turn
    if curved mode enabled:
        apply cubic curve to left
        apply cubic curve to right
    move left motors at left
    move right motors at right

static int cubicCurve(v):
    // Apply cubic input shaping for smoother control
    return (v * v * v) / (127 * 127)
    // Small inputs become smaller, max ±127 stays at ±127


LOW-LEVEL CONTROL
-----------------

void setOpenLoop(left, right):
    // Direct motor control with velocity [-127, 127]
    clamp left to range [-127, 127]
    clamp right to range [-127, 127]
    move left motors at left
    move right motors at right

void setVoltage(leftMv, rightMv):
    // Direct voltage control in millivolts [-12000, 12000]
    clamp leftMv to range [-12000, 12000]
    clamp rightMv to range [-12000, 12000]
    apply leftMv to left motors
    apply rightMv to right motors

void tare():
    // Reset all motor encoders to zero
    reset position of all motors to 0

void brake():
    // Stop and brake all motors
    brake left motors
    brake right motors


CONFIGURATION
-------------

void setBrakeMode(mode):
    // Set brake behavior (COAST/BRAKE/HOLD)
    set left motors to mode
    set right motors to mode
    set all motors to mode

void setEncoderUnits(units):
    // Configure encoder measurement (DEGREES/ROTATIONS/COUNTS)
    set left motors to units
    set right motors to units


SENSORS
-------

void getPosition(leftTicks, rightTicks):
    // Read current encoder positions
    read left motors position → leftTicks
    read right motors position → rightTicks

vector<double> getTemperatures():
    // Read all motor temperatures
    query all motors for temperature
    return vector of temperatures


MOTION PRIMITIVES (Blocking)
-----------------------------

void move(distInches, velocity):
    // Move straight forward/backward by distance
    reset all motor encoders to 0
    ticks = distInches / distPerTick
    command all motors to move ticks at velocity
    while not within ±5 ticks of target:
        check current position
        delay 2ms
    brake all motors
    delay 150ms

void turn(thetaDeg, velocity):
    // Turn in place by angle
    reset left and right encoders to 0
    arcDistance = (thetaDeg / 360) * π * wheelBase
    ticks = arcDistance / distPerTick
    right motors move -ticks  // backward
    left motors move +ticks   // forward
    while not within ±5 ticks on both sides:
        check left and right positions
        delay 2ms
    brake both sides
    delay 150ms

================================================================================
*/

void Drivetrain::tank(int leftY, int rightY) {
    if (curved_) {
        left_.move(cubicCurve(leftY) * DRIFT_COMPENSATION);
        right_.move(cubicCurve(rightY));
    } else {
        left_.move(leftY * DRIFT_COMPENSATION);
        right_.move(rightY);
    }
}

void Drivetrain::split(int power, int turn) {
    int l = power + turn;
    int r = power - turn;
    if (curved_) { l = cubicCurve(l); r = cubicCurve(r); }
    left_.move(l * DRIFT_COMPENSATION);
    right_.move(r);
}

void Drivetrain::arcade(int power, int turn) {
    int l = power + turn;
    int r = power - turn;
    if (curved_) { l = cubicCurve(l); r = cubicCurve(r); }
    left_.move(l * DRIFT_COMPENSATION);
    right_.move(r);
}

void Drivetrain::setOpenLoop(int left, int right) {
    // Clamp to [-127, 127]
    if (left > 127) left = 127; if (left < -127) left = -127;
    if (right > 127) right = 127; if (right < -127) right = -127;
    left_.move(left * DRIFT_COMPENSATION);
    right_.move(right);
}

void Drivetrain::setVoltage(int leftMv, int rightMv) {
    if (leftMv > 12000) leftMv = 12000; if (leftMv < -12000) leftMv = -12000;
    if (rightMv > 12000) rightMv = 12000; if (rightMv < -12000) rightMv = -12000;
    left_.move_voltage(leftMv * DRIFT_COMPENSATION);
    right_.move_voltage(rightMv);
}

void Drivetrain::tare() {
    all_.tare_position();
}

void Drivetrain::brake() {
    left_.brake();
    right_.brake();
}

void Drivetrain::setBrakeMode(pros::motor_brake_mode_e_t mode) {
    left_.set_brake_mode_all(mode);
    right_.set_brake_mode_all(mode);
    all_.set_brake_mode_all(mode);
}

void Drivetrain::setEncoderUnits(pros::motor_encoder_units_e_t units) {
    left_.set_encoder_units_all(units);
    right_.set_encoder_units_all(units);
}

void Drivetrain::getPosition(double& leftTicks, double& rightTicks) const {
    leftTicks = left_.get_position();
    rightTicks = right_.get_position();
}

std::vector<double> Drivetrain::getTemperatures() const {
    return all_.get_temperature_all();
}

void Drivetrain::move(double distInches, int velocity) {
    all_.tare_position();
    const double ticks = distInches / distPerTick_;
    all_.move_relative(ticks, velocity);

    // Wait until within +/-5 ticks of target
    while (!((all_.get_position() < ticks + 5) && (all_.get_position() > ticks - 5))) {
        pros::delay(2);
    }

    all_.brake();
    pros::delay(150);
}

void Drivetrain::turn(double thetaDeg, int velocity) {
    right_.tare_position();
    left_.tare_position();

    const double ticks = ((thetaDeg / 360.0) * M_PI * wheelBase_) / distPerTick_;

    right_.move_relative(-ticks, velocity);
    left_.move_relative(ticks, velocity);

    while (!((right_.get_position() > ticks - 5) && (right_.get_position() < ticks + 5) &&
             (left_.get_position()  > ticks - 5) && (left_.get_position()  < ticks + 5))) {
        pros::delay(2);
    }

    right_.brake();
    left_.brake();
    pros::delay(150);
}
