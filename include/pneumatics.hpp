#pragma once
#include "globals.hpp"

// ----------------------------------------------------------------------------
// Pneumatics subsystem (OOP)
// ----------------------------------------------------------------------------
class Pneumatics {
public:
	explicit Pneumatics(pros::Controller& controller) : controller_(controller) {}

	// Toggle a pin when a controller button is pressed; returns new state
	bool toggle(bool state, pros::controller_digital_e_t button, pros::adi::Port& pin);

	// Set a pin state directly (autonomous helper)
	void set(bool state, pros::adi::Port& pin);

private:
	pros::Controller& controller_;
};

// Legacy wrappers (kept for compatibility during migration)
bool switchState(bool state, pros::controller_digital_e_t button, pros::adi::Port pin);
void setAutonPin(bool state, pros::adi::Port pin);