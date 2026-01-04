#pragma once
#include "main.h"

// Autonomous selector for controller-based path selection
// Displays menu on controller screen with cursor navigation

// Path names for display (defined in autonSelector.cpp)
extern const char* pathNames[6];

/**
 * Run autonomous selector on controller screen
 * User navigates with UP/DOWN buttons, confirms with A
 * Returns: selected path number (1-6)
 */
int selectAutonomousPath();

/**
 * Display the current selection state on controller
 * Shows all 6 paths with cursor indicator
 */
void displayPathSelection(int currentSelection);
