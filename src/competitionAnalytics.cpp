#include "competitionAnalytics.hpp"
#include "globals.hpp"
#include "autonSelector.hpp"
#include <cmath>

// Task handle for background update
static pros::Task* analyticsTask = nullptr;
static bool taskRunning = false;

// Screen dimensions
constexpr int SCREEN_WIDTH = 480;
constexpr int SCREEN_HEIGHT = 240;
constexpr int HEADER_HEIGHT = 30;
constexpr int MOTOR_BAR_MARGIN = 10;

// Temperature thresholds (Celsius)
constexpr double TEMP_COOL = 30.0;      // Below this = green
constexpr double TEMP_WARM = 45.0;      // Below this = yellow
constexpr double TEMP_HOT = 55.0;       // Above this = red
constexpr double TEMP_MAX = 70.0;       // Max scale for bar height

uint32_t getTemperatureColor(double temp) {
    if (temp < TEMP_COOL) {
        // Cool - Green
        return static_cast<uint32_t>(pros::Color::green);
    } else if (temp < TEMP_WARM) {
        // Warming - Yellow/Orange
        return static_cast<uint32_t>(pros::Color::yellow);
    } else if (temp < TEMP_HOT) {
        // Hot - Orange
        return static_cast<uint32_t>(pros::Color::orange);
    } else {
        // Very hot - Red
        return static_cast<uint32_t>(pros::Color::red);
    }
}

void drawMotorTempBar(const char* motorName, pros::Motor& motor, int x, int y, int width, int height) {
    // Get motor temperature
    double temp = motor.get_temperature();
    
    // Handle error case
    if (temp == PROS_ERR_F) {
        temp = 0.0;
    }
    
    // Calculate bar fill height (0-100% based on 0-70°C)
    double tempPercent = (temp / TEMP_MAX) * 100.0;
    if (tempPercent > 100.0) tempPercent = 100.0;
    if (tempPercent < 0.0) tempPercent = 0.0;
    
    int fillHeight = static_cast<int>((tempPercent / 100.0) * height);
    
    // Draw background (empty bar)
    pros::screen::set_pen(pros::Color::black);
    pros::screen::fill_rect(x, y, x + width, y + height);
    
    // Draw border
    pros::screen::set_pen(pros::Color::white);
    pros::screen::draw_rect(x, y, x + width, y + height);
    
    // Draw filled portion (from bottom up)
    uint32_t barColor = getTemperatureColor(temp);
    pros::screen::set_pen(barColor);
    int fillY = y + height - fillHeight;
    pros::screen::fill_rect(x + 2, fillY, x + width - 2, y + height - 2);
    
    // Draw motor name below bar
    pros::screen::set_pen(pros::Color::white);
    int textX = x + (width / 2) - 20;  // Center text approximately
    int textY = y + height + 5;
    pros::screen::print(pros::E_TEXT_SMALL, textX, textY, "%s", motorName);
    
    // Draw temperature value
    int tempTextX = x + (width / 2) - 15;
    int tempTextY = y - 15;
    pros::screen::print(pros::E_TEXT_SMALL, tempTextX, tempTextY, "%.0f C", temp);
}

void initAnalyticsDisplay() {
    pros::screen::set_eraser(pros::Color::black);
    pros::screen::erase();
}

void updateAnalyticsDisplay() {
    // Clear screen
    pros::screen::set_eraser(pros::Color::black);
    pros::screen::erase();
    
    // Draw header background
    pros::screen::set_pen(pros::Color::blue);
    pros::screen::fill_rect(0, 0, SCREEN_WIDTH, HEADER_HEIGHT);
    
    // Top Left: Selected autonomous path
    pros::screen::set_pen(pros::Color::white);
    if (autonSelected && autonID >= '1' && autonID <= '6') {
        int pathIndex = autonID - '1';  // Convert '1'-'6' to 0-5
        pros::screen::print(pros::E_TEXT_MEDIUM, 5, 5, "Auton: %s", pathNames[pathIndex]);
    } else {
        pros::screen::print(pros::E_TEXT_MEDIUM, 5, 5, "Auton: None");
    }
    
    // Top Right: Battery percentage
    int32_t batteryCapacity = pros::battery::get_capacity();
    pros::Color batteryColor;
    if (batteryCapacity > 75) {
        batteryColor = pros::Color::green;
    } else if (batteryCapacity > 50) {
        batteryColor = pros::Color::yellow;
    } else if (batteryCapacity > 25) {
        batteryColor = pros::Color::orange;
    } else {
        batteryColor = pros::Color::red;
    }
    
    pros::screen::set_pen(batteryColor);
    pros::screen::print(pros::E_TEXT_MEDIUM, SCREEN_WIDTH - 100, 5, "Bat: %d%%", (int)batteryCapacity);
    
    // Motor temperature bars
    // Calculate bar dimensions
    int numMotors = 9;  // 6 drive + 3 intake
    int totalMargins = MOTOR_BAR_MARGIN * (numMotors + 1);
    int availableWidth = SCREEN_WIDTH - totalMargins;
    int barWidth = availableWidth / numMotors;
    int barHeight = SCREEN_HEIGHT - HEADER_HEIGHT - 60;  // Leave room for labels
    int barY = HEADER_HEIGHT + 25;
    
    // Draw all motor bars
    int currentX = MOTOR_BAR_MARGIN;
    
    // Get motor references from globals
    pros::Motor leftMotors[3] = {
        pros::Motor(8),   // Left front
        pros::Motor(6),   // Left middle  
        pros::Motor(10)   // Left back
    };
    
    pros::Motor rightMotors[3] = {
        pros::Motor(9),   // Right front
        pros::Motor(7),   // Right middle
        pros::Motor(5)    // Right back
    };
    
    // Draw left drive motors
    drawMotorTempBar("L1", leftMotors[0], currentX, barY, barWidth, barHeight);
    currentX += barWidth + MOTOR_BAR_MARGIN;
    
    drawMotorTempBar("L2", leftMotors[1], currentX, barY, barWidth, barHeight);
    currentX += barWidth + MOTOR_BAR_MARGIN;
    
    drawMotorTempBar("L3", leftMotors[2], currentX, barY, barWidth, barHeight);
    currentX += barWidth + MOTOR_BAR_MARGIN;
    
    // Draw right drive motors
    drawMotorTempBar("R1", rightMotors[0], currentX, barY, barWidth, barHeight);
    currentX += barWidth + MOTOR_BAR_MARGIN;
    
    drawMotorTempBar("R2", rightMotors[1], currentX, barY, barWidth, barHeight);
    currentX += barWidth + MOTOR_BAR_MARGIN;
    
    drawMotorTempBar("R3", rightMotors[2], currentX, barY, barWidth, barHeight);
    currentX += barWidth + MOTOR_BAR_MARGIN;
    
    // Draw intake motors
    drawMotorTempBar("Low", lowIntakeMotor, currentX, barY, barWidth, barHeight);
    currentX += barWidth + MOTOR_BAR_MARGIN;
    
    drawMotorTempBar("Mid", midIntakeMotor, currentX, barY, barWidth, barHeight);
    currentX += barWidth + MOTOR_BAR_MARGIN;
    
    drawMotorTempBar("Hi", highIntakeMotor, currentX, barY, barWidth, barHeight);
}

// Background task function
void analyticsTaskFunc(void* param) {
    while (taskRunning) {
        updateAnalyticsDisplay();
        pros::delay(10000);  // Update every 10 seconds
    }
}

void startAnalyticsTask() {
    if (!taskRunning) {
        taskRunning = true;
        initAnalyticsDisplay();
        analyticsTask = new pros::Task(analyticsTaskFunc);
    }
}

void stopAnalyticsTask() {
    if (taskRunning) {
        taskRunning = false;
        if (analyticsTask != nullptr) {
            delete analyticsTask;
            analyticsTask = nullptr;
        }
    }
}
