#pragma once
#include "globals.hpp"

// ----------------------------------------------------------------------------
// Intake subsystem (OOP) - Three-stage intake system
// ----------------------------------------------------------------------------
class Intake {
public:
	Intake(pros::Motor& lowMotor, pros::Motor& midMotor, pros::Motor& highMotor, pros::Controller& controller)
		: lowMotor_(lowMotor), midMotor_(midMotor), highMotor_(highMotor), controller_(controller) {}

	// Intake modes
	enum class IntakeMode {
		STOP,
		INTAKE,          // Low forward only
		SCORE_LOW,       // Low backward (reverse/outtake)
		SCORE_MID,       // Mid forward + High forward
		SCORE_HIGH,      // Mid forward + High backward
		STORE            // Mid forward + High forward (same as SCORE_MID)
	};

	// Teleop control using controller buttons
	void teleopControl();

	// Set intake mode (for autonomous or manual control)
	void setMode(IntakeMode mode);

	// Direct control of individual motors [-127, 127]
	void runLow(int power);
	void runMid(int power);
	void runHigh(int power);

	// Stop all motors
	void stopAll();

	// Autonomous helper: run a mode for N seconds
	void autonRunSeconds(IntakeMode mode, int seconds);

	// Helpers for migration/tests
	void tare();
	void setBrakeMode(pros::motor_brake_mode_e_t mode);

private:
	pros::Motor& lowMotor_;
	pros::Motor& midMotor_;
	pros::Motor& highMotor_;
	pros::Controller& controller_;
};

// Legacy wrappers (kept for compatibility during migration)
void intake();
void autonIntake(int time);