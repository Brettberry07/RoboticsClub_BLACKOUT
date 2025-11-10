#pragma once
#include "globals.hpp"
#include <cmath>

// ----------------------------------------------------------------------------
// Drivetrain: encapsulates control modes and motion primitives
// ----------------------------------------------------------------------------
class Drivetrain {
public:
    Drivetrain(pros::MotorGroup& left,
               pros::MotorGroup& right,
               pros::MotorGroup& all,
               double distPerTickIn,
               double wheelBaseIn)
        : left_(left), right_(right), all_(all),
          distPerTick_(distPerTickIn), wheelBase_(wheelBaseIn) {}

    // Runtime control modes
    void setCurved(bool curved) { curved_ = curved; }
    bool isCurved() const { return curved_; }

    void tank(int leftY, int rightY);
    void split(int power, int turn);
    void arcade(int power, int turn);
    // Direct open-loop left/right drive [-127..127]
    void setOpenLoop(int left, int right);
    // Direct voltage control [-12000..12000] per side
    void setVoltage(int leftMv, int rightMv);
    // Utilities
    void tare();          // Tare all drivetrain encoders
    void brake();         // Brake both sides
    
    // Configuration
    void setBrakeMode(pros::motor_brake_mode_e_t mode);
    void setEncoderUnits(pros::motor_encoder_units_e_t units);
    
    // Sensors
    void getPosition(double& leftTicks, double& rightTicks) const;
    std::vector<double> getTemperatures() const;

    // Motion primitives (blocking until within tolerance)
    void move(double distInches, int velocity);
    void turn(double thetaDeg, int velocity);

    // Utility: cubic input shaping [-127..127]
    static inline int cubicCurve(int v) {
        return (v * v * v) / (127 * 127);
    }

private:
    pros::MotorGroup& left_;
    pros::MotorGroup& right_;
    pros::MotorGroup& all_;

    bool curved_ = true;
    double distPerTick_;
    double wheelBase_;
};
