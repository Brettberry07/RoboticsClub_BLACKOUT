#include "drivetrain_oop.hpp"

void Drivetrain::tank(int leftY, int rightY) {
    if (curved_) {
        left_.move(cubicCurve(leftY));
        right_.move(cubicCurve(rightY));
    } else {
        left_.move(leftY);
        right_.move(rightY);
    }
}

void Drivetrain::split(int power, int turn) {
    int l = power + turn;
    int r = power - turn;
    if (curved_) { l = cubicCurve(l); r = cubicCurve(r); }
    left_.move(l);
    right_.move(r);
}

void Drivetrain::arcade(int power, int turn) {
    int l = power + turn;
    int r = power - turn;
    if (curved_) { l = cubicCurve(l); r = cubicCurve(r); }
    left_.move(l);
    right_.move(r);
}

void Drivetrain::setOpenLoop(int left, int right) {
    // Clamp to [-127, 127]
    if (left > 127) left = 127; if (left < -127) left = -127;
    if (right > 127) right = 127; if (right < -127) right = -127;
    left_.move(left);
    right_.move(right);
}

void Drivetrain::setVoltage(int leftMv, int rightMv) {
    if (leftMv > 12000) leftMv = 12000; if (leftMv < -12000) leftMv = -12000;
    if (rightMv > 12000) rightMv = 12000; if (rightMv < -12000) rightMv = -12000;
    left_.move_voltage(leftMv);
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
