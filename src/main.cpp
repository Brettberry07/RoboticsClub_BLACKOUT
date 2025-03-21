#include "globals.hpp"
#include <numeric>
#include "pros/apix.h"
#include "pros/rtos.hpp"
#include "liblvgl/lvgl.h"


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::screen::set_pen(pros::Color::white);
	pros::screen::set_eraser(pros::Color::black);

	imuSensor.reset(true);// Initialize the IMU sensor
	imuSensor.tare(); // Reset the IMU's position to 0 degrees
	imuSensor.set_yaw(0); // Reset the IMU's position to 0 degrees

	// for(int i=0; i<6; i++){
	// 	drawButton(buttons[i]);
	// }

	setAutonPin(HIGH, clampPin);

	//setting encoder units
	leftChassis.set_encoder_units_all(pros::E_MOTOR_ENCODER_COUNTS);
	rightChassis.set_encoder_units_all(pros::E_MOTOR_ENCODER_COUNTS);
	driveTrainMotors.set_encoder_units_all(pros::E_MOTOR_ENCODER_COUNTS);

	ladyBrownMotors.set_encoder_units_all(pros::E_MOTOR_ENCODER_COUNTS);

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
	low_profile_check();

	// while(true){
	// 	if(autonSelected){
	// 		break;
	// 	}
	// 	for(int i=0; i<6; i++){
	// 		drawButton(buttons[i]);
	// 	}
		
	// 	// Check if the screen has been touched
	// 	// if it has been touched, get the button title for the button that was pressed
	// 	status = pros::screen::touch_status();
	// 	if (status.touch_status && !autonSelected){
	// 		for(int i=0; i<6; i++){
	// 			if(buttonTouched(buttons[i], status.x, status.y)){
	// 				buttons[i].isPressed = !buttons[i].isPressed;
	// 				autonID = buttons[i].title;
	// 				autonSelected = true;
	// 			}
	// 		}
	// 	}
	// 	// Set a delay after a button has been pressed for user experience
	// 	pros::delay(20);
	// }
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not pick up 
 * from where it left off.
 */
void autonomous() {
	// Switch statment to select the auton path
	switch(autonID) {
		// Button number 1: Start bottom right - top left
		case '1':
			topLeft();
			pros::screen::fill_rect(0, 0, 480, 136);
			break;
		// Button Number 2: Start bottom left - top right
		case '2':
			topRight();
			pros::screen::fill_rect(0, 0, 480, 136);
			break;
		// Button number 3: nothing atm
		case '3':
			full_system_check();
			pros::screen::fill_rect(0, 0, 480, 136);
			break;
		// Button number 4: Start bottom left - top right
		case '4':
			bottomLeft();
			pros::screen::fill_rect(0, 0, 480, 136);
			break;
		// Button number 5: Start bottom right - top left
		case '5':
			bottomRight();
			pros::screen::fill_rect(0, 0, 480, 136);
			break;
		// Button number 6: nothing atm
		case '6':	
			testAuton();
			pros::screen::fill_rect(0, 0, 480, 136);
			break;
		// Defualt case: Should not be reached
		default:
			// redRingRush();
			// blueRingRush();
			// redGoalRush();
			blueGoalRush();
			// newAutonSkills();
			// autonSkills();

			pros::screen::fill_rect(0, 0, 480, 136);
			break;
	}
}


/**
 * @brief Operator control function for the robot.
 * 
 * This function is called during the operator control period of the robot's operation.
 * It sets the brake modes for various motor groups, monitors motor temperatures, and 
 * provides feedback through the controller's rumble feature if the average temperature 
 * exceeds a threshold. It also prints IMU sensor data to the screen and calls the 
 * testAuton function in a loop.
 * 
 * The function performs the following tasks:
 * - Sets brake modes for drive train, left chassis, right chassis, and intake motors.
 * - Monitors motor temperatures and triggers a rumble pattern on the controller if the 
 *   average temperature exceeds 45 degrees Celsius.
 * - Prints IMU sensor data (heading, yaw, pitch, roll) to the screen.
 * - Calls the driveTrain and intake functions to control the robot's movement.
 * - Delays the loop to prevent CPU overflow.
 */

void opcontrol() {
	driveTrainMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	leftChassis.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
	rightChassis.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
	intakeMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

	bool excededMaxTemp = false;
	int count = 0;
	const char* tempRumble = "--- .... ---"; // "-" is long, "." is short, " " is a pause

	while(excededMaxTemp == false) {
		if(count % 1000 == 0)
		{
			std::vector<double> allTemps = driveTrainMotors.get_temperature_all();
			double averageTemps = std::accumulate(allTemps.begin(), allTemps.end(), 0.0) / allTemps.size();

			std::vector<double> intakeTemps = intakeMotors.get_temperature_all();
			double averageIntakeTemp = std::accumulate(intakeTemps.begin(), intakeTemps.end(), 0.0) / intakeTemps.size();

			if(averageTemps >= 55 || averageIntakeTemp >= 55) {
				if(master.rumble(tempRumble) != 1) {
					pros::delay(50); // PROS only updates Controller every 50ms 
					master.rumble(tempRumble);
					excededMaxTemp = true;
				}
			}
		}
		count += 1;

		clampPneumaticsState = switchState(clampPneumaticsState, pros::E_CONTROLLER_DIGITAL_L2, clampPin);
		clampPneumaticsState = switchState(clampPneumaticsState, pros::E_CONTROLLER_DIGITAL_L1, clampPin);

		driveTrain('t', isCurved, driveOrIntakeState);
		intake();
		ladybrown();
		
		if(test_batteries() && real_time_test()) {
			pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Battery Level Nominal");
		} else {
			pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Battery Level Low");
			
		}
		pros::delay(10); // We do not want the CPU to overflow with too many commands
	}
}
