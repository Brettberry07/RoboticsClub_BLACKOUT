#pragma once
#include "main.h"

// Competition analytics display for brain screen
// Shows motor temperatures, battery %, and selected autonomous

/**
 * Initialize analytics display
 * Sets up screen and prepares for rendering
 */
void initAnalyticsDisplay();

/**
 * Update and render analytics display
 * Call this repeatedly during competition (in loop or task)
 * Shows:
 * - Selected autonomous path (top left)
 * - Battery percentage (top right)
 * - Motor temperature bars (main area)
 */
void updateAnalyticsDisplay();

/**
 * Draw a single motor temperature bar
 * @param motorName Name of motor to display
 * @param motor Motor object to read temperature from
 * @param x X position (left edge)
 * @param y Y position (top edge)
 * @param width Bar width in pixels
 * @param height Bar height in pixels
 */
void drawMotorTempBar(const char* motorName, pros::Motor& motor, int x, int y, int width, int height);

/**
 * Get color based on temperature
 * @param temp Temperature in degrees Celsius
 * @return Color code (green -> yellow -> red based on temp)
 */
uint32_t getTemperatureColor(double temp);

/**
 * Start analytics display task
 * Runs display update in background during competition
 */
void startAnalyticsTask();

/**
 * Stop analytics display task
 */
void stopAnalyticsTask();
