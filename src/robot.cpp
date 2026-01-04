#include "robot.hpp"

// Construct subsystems using existing globals from globals.cpp
static Drivetrain g_drivetrain(leftChassis, rightChassis, driveTrainMotors, distOneTick, wheelBase);
static Odometry   g_odometry(imuSensor, verticalSensor, horizontalSensor, wheelBase, distOneTick);
static Intake     g_intake(lowIntakeMotor, midIntakeMotor, highIntakeMotor, master);
static Pneumatics g_pneumatics(master);
static Robot      g_robot(g_drivetrain, g_odometry, g_intake, g_pneumatics);

Robot& getRobot() { return g_robot; }
