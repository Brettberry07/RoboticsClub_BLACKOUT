#include "globals.hpp"
#include "robot.hpp"
#include <numeric>
#include "pros/apix.h"
#include "pros/rtos.hpp"
#include "liblvgl/lvgl.h"
#include "pathFollower.hpp"
#include "pathFollowerTests.hpp"
#include "autonomousPathExample.hpp"


/**
 * Initialization
 * --------------
 * Runs as soon as the program starts. All other competition modes are blocked
 * by initialize(); keep execution time under a few seconds.
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

	getRobot().pneumatics.set(HIGH, clampPin);

	//setting encoder units
	getRobot().drivetrain.setEncoderUnits(pros::E_MOTOR_ENCODER_COUNTS);

	// setting brake modes for drivetrain and intake via OOP APIs
	getRobot().drivetrain.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	getRobot().intake.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
}



/**
 * Disabled
 * --------
 * Runs while the robot is disabled (after autonomous or opcontrol). Exits when
 * the robot is enabled again.
 */
void disabled() {}

/**
 * Competition Initialize
 * ----------------------
 * Runs after initialize() and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. Intended for competition-
 * specific setup (e.g., autonomous selector). Exits when enabled and a mode
 * starts.
 */
void competition_initialize() {

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
 * Autonomous
 * ----------
 * Runs user autonomous code. Starts in its own task when the robot is enabled
 * in autonomous mode. May also be invoked from initialize() or opcontrol() for
 * non-competition testing.
 *
 * If disabled or communications are lost, this task stops. Re-enabling restarts
 * the task (does not resume from prior state).
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
			// Path Follower Setup and Test
			// Initialize robot position and sensors
			// imuSensor.reset();
			getRobot().drivetrain.tare();
			getRobot().odometry.reset(0.0, 0.0, 0.0); // Start pose
			
			// pros::delay(2000);  // Wait for IMU to calibrate

			// linearPID(24);
			
			pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Starting Path Follower Test");
			
			// Run the path follower test
			testPathFollower();
			
			// Alternative tests you can uncomment:
			// testMoveTo();               // Simple point-to-point movement
			// testMoveToWithHeading();    // Move with final heading
			// testIndividualCurve();      // Test one curve at a time
			// tunePathFollower();         // Experiment with parameters
			
			// Or use the example autonomous routines:
			// autonomousFollowPath();     // Follow complete JSON path
			// autonomousMultiplePoints(); // Navigate multiple points
			// autonomousPrecisionPath();  // Precision tuning
			// autonomousFastPath();       // Fast tuning
			// autonomousTestFirstCurve(); // Test just first curve

			pros::screen::fill_rect(0, 0, 480, 136);
			break;
	}
}


/**
 * Operator Control
 * ----------------
 * Called during the operator period. Responsibilities:
 * - Set brake modes for drivetrain, left/right chassis, and intake motors.
 * - Monitor motor temperatures; trigger controller rumble if average exceeds
 *   the threshold.
 * - Optionally print IMU data to the screen (heading, yaw, pitch, roll).
 * - Call testAuton() as needed.
 * - Delay loop to prevent CPU overload.
 */

void opcontrol() {
	// setAutonPin(HIGH, clampPin);
	getRobot().drivetrain.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	getRobot().intake.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

	int count = 0;
	const char* rumble_pattern = "- .... -"; // "-" is long, "." is short, " " is a pause
	// full_system_check();
	while(true){
		if(count % 1000 == 0)
		{
			std::vector<double> allTemps = getRobot().drivetrain.getTemperatures();
			double averageTemps = std::accumulate(allTemps.begin(), allTemps.end(), 0.0) / allTemps.size();
			if(averageTemps >= 55) {
				if(master.rumble(rumble_pattern) != 1) {
					pros::delay(50); // PROS only updates Controller every 50ms 
					master.rumble(rumble_pattern);
				}
			}
		}
		count += 1;

	clampPneumaticsState = getRobot().pneumatics.toggle(clampPneumaticsState, pros::E_CONTROLLER_DIGITAL_L2, clampPin);
	clampPneumaticsState = getRobot().pneumatics.toggle(clampPneumaticsState, pros::E_CONTROLLER_DIGITAL_L1, clampPin);

		// Toggle curve mode on Y (rising edge) and drive with OOP drivetrain (Tank)
		static bool prevY = false;
		bool yPressed = master.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
		if (yPressed && !prevY) {
			isCurved = !isCurved;
		}
		prevY = yPressed;

		getRobot().drivetrain.setCurved(isCurved);
		int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

	getRobot().drivetrain.tank(leftY, rightY);
	getRobot().intake.teleopControl();

		// pros::screen::print(pros::E_TEXT_MEDIUM, 1, "%d", imuSensor.get_heading());
		
		pros::delay(10); // We do not want the CPU to overflow
	}
}
