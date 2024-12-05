#include "globals.hpp"
#include "pros/apix.h"
#include "pros/rtos.hpp"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::screen::set_pen(pros::Color::white);
	pros::screen::set_eraser(pros::Color::black);

	for(int i=0; i<6; i++){
		drawButton(buttons[i]);
	}

	//setting encoder units
	leftChassis.set_encoder_units_all(pros::E_MOTOR_ENCODER_COUNTS);
	rightChassis.set_encoder_units_all(pros::E_MOTOR_ENCODER_COUNTS);

	//setting brake modes
	driveTrainMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	leftChassis.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	rightChassis.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

	intakeMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {

	while(true){
		if(autonSelected){
			break;
		}
		for(int i=0; i<6; i++){
			drawButton(buttons[i]);
		}
		
		// Check if the screen has been touched
		// if it has been touched, get the button title for the button that was pressed
		status = pros::screen::touch_status();
		if (status.touch_status && !autonSelected){
			for(int i=0; i<6; i++){
				if(buttonTouched(buttons[i], status.x, status.y)){
					buttons[i].isPressed = !buttons[i].isPressed;
					autonID = buttons[i].title;
					autonSelected = true;
				}
			}
		}
		// Set a delay after a button has been pressed for user experience
		pros::delay(20);
	}
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	// Switch statment to select the auton path
	switch(autonID) {
		// Button number 1: Start bottom right - top left
		case '1':
			bottomRight_TopLeft();
			pros::screen::fill_rect(0, 0, 480, 136);
			break;
		// Button Number 2: Start bottom left - top right
		case '2':
			bottomLeft_TopRight();
			pros::screen::fill_rect(0, 0, 480, 136);
			break;
		// Button number 3: nothing atm
		case '3':
			testAuton();
			pros::screen::fill_rect(0, 0, 480, 136);
			break;
		// Button number 4: Start bottom left - top right
		case '4':
			bottomLeft_TopRight();
			pros::screen::fill_rect(0, 0, 480, 136);
			break;
		// Button number 5: Start bottom right - top left
		case '5':
			bottomRight_TopLeft();
			pros::screen::fill_rect(0, 0, 480, 136);
			break;
		// Button number 6: nothing atm
		case '6':	
			testAuton();
			pros::screen::fill_rect(0, 0, 480, 136);
			break;
		// Defualt case: Should not be reached
		default:
			testAuton();
			pros::screen::fill_rect(0, 0, 480, 136);
			break;
	}
	
	
	// TODO: uncomment the one that we need:
	// NOT ANY MORE WOOOOOOOOOOOOO
	// bottomLeft_TopRight();
	// bottomRight_TopLeft();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	// setAutonPin(HIGH, clampPin);
	driveTrainMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	leftChassis.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
	rightChassis.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
	intakeMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);



	while(true){
		pros::screen::set_pen(pros::Color::white);
		pros::screen::print(TEXT_MEDIUM, 1, "hello");
		clampPneumaticsState = switchState(clampPneumaticsState, pros::E_CONTROLLER_DIGITAL_A, clampPin);
		driveTrain('t', isCurved, driveOrIntakeState);
		intake();
		// testAuton();
		// driveTrainMotors.move_velocity(100);

		pros::delay(10);
	}
}
