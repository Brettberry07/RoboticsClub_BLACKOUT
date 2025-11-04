#pragma once
#include "globals.hpp"
#include "drivetrain_oop.hpp"
#include "odometry.hpp"
#include "intake.hpp"
#include "pneumatics.hpp"

// ----------------------------------------------------------------------------
// Robot: holds composed subsystems and provides a single access point
// ----------------------------------------------------------------------------
class Robot {
public:
    Robot(Drivetrain& dt, Odometry& od, Intake& in, Pneumatics& pn)
        : drivetrain(dt), odometry(od), intake(in), pneumatics(pn) {}

    Drivetrain& drivetrain;
    Odometry& odometry;
    Intake& intake;
    Pneumatics& pneumatics;
};

// Global accessor to a single robot instance backed by existing globals
Robot& getRobot();
