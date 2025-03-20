#include "globals.hpp"

// PID constants for ladyBrown motors
PIDConstants ladyBrownPID_Constants = {100, 125, 75};

void ladyBrownPID(double target) {
    int32_t power = 0;
    double ladyBrownTicks = 0;

    // set the brake to hold it in place
    ladyBrownMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // Reset chassis positions
    ladyBrownMotors.tare_position();
    
    // Reset PID state
    ladyBrownPID_Constants.error = 0;
    ladyBrownPID_Constants.prevError = 0;
    ladyBrownPID_Constants.integral = 0;

    uint32_t previousTime = pros::millis();
    double totalTime = 0;

    while (true) {
        ladyBrownTicks = ladyBrownMotors.get_position();
        ladyBrownPID_Constants.error = target - ladyBrownTicks; // Assuming you want to reach a specific position
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Error: %f", ladyBrownPID_Constants.error);

        uint32_t currentTime = pros::millis();
        double dt = (currentTime - previousTime) / 1000.0; // dt in seconds
        totalTime += dt;

        // Use dt in derivative and integral calculations
        ladyBrownPID_Constants.derivative = (ladyBrownPID_Constants.error - ladyBrownPID_Constants.prevError) / dt;
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Derivative: %f", ladyBrownPID_Constants.derivative);

        ladyBrownPID_Constants.integral += ladyBrownPID_Constants.error * dt;
        ladyBrownPID_Constants.integral = std::clamp(ladyBrownPID_Constants.integral, ladyBrownPID_Constants.low, ladyBrownPID_Constants.high);
        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Integral: %f", ladyBrownPID_Constants.integral);

        // Compute control signal (power)
        power = (ladyBrownPID_Constants.kP * ladyBrownPID_Constants.error) + 
                (ladyBrownPID_Constants.kI * ladyBrownPID_Constants.integral) + 
                (ladyBrownPID_Constants.kD * ladyBrownPID_Constants.derivative);
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Power: %d", power);

        // Check for timeout (using currentTime if you want an absolute time based on millis)
        if ((totalTime) >= ladyBrownPID_Constants.timeOut) {
            pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Time Out, Time reached: %f", ladyBrownPID_Constants.timeOut);
            break;
        } 

        // Exit if error is within acceptable threshold (1 degree)
        if (std::abs(ladyBrownPID_Constants.error) < 1) {
            pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Min range met. Range: %f", ladyBrownPID_Constants.error);
            break;
        }

        // Apply motor commands (differential drive: one side reversed)
        ladyBrownMotors.move_voltage(power);

        ladyBrownPID_Constants.prevError = ladyBrownPID_Constants.error;
        previousTime = currentTime;

        pros::delay(10);
    }

    ladyBrownMotors.brake();
    pros::delay(100); // Allow some time for motors to stop completely
}

// Function for operating ladyBrown motors using macros
void ladybrown(){
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A)){  //raise
        ladyBrownPID(100);
    }
    else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){  //lower
        ladyBrownPID(-100);
    }
    else{
        ladyBrownMotors.brake(); // stop the motors when no button is pressed
    }
}