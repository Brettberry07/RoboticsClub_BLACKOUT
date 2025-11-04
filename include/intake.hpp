#pragma once
#include "globals.hpp"

// ----------------------------------------------------------------------------
// Intake subsystem (OOP)
// ----------------------------------------------------------------------------
class Intake {
public:
	Intake(pros::MotorGroup& motors, pros::Controller& controller)
		: motors_(motors), controller_(controller) {}

	// Teleop control using controller buttons R1/R2
	void teleopControl();

	// Run intake at a specified power [-127, 127]
	void run(int power);

	// Autonomous helper: run for N seconds
	void autonRunSeconds(int seconds);

	// Helpers for migration/tests
	void tare();
	void setVoltage(int mV);
	int getPosition() const;

	// Configure brake mode on intake motors
	void setBrakeMode(pros::motor_brake_mode_e_t mode);

private:
	pros::MotorGroup& motors_;
	pros::Controller& controller_;
};

// Legacy wrappers (kept for compatibility during migration)
void intake();
void autonIntake(int time);