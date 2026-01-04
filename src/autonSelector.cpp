#include "autonSelector.hpp"
#include "globals.hpp"

// Define path names here (declared extern in header)
const char* pathNames[6] = {
    "Path 1",
    "Path 2", 
    "Path 3",
    "Path 4",
    "Path 5",
    "Path 6"
};

void displayPathSelection(int currentSelection) {
    // Clear controller screen
    master.clear();
    
    // Display each path option with cursor indicator
    for (int i = 0; i < 6; i++) {
        // Create line text with cursor for selected item
        char line[20];
        if (i == currentSelection) {
            // Add cursor (arrow) for selected path
            snprintf(line, sizeof(line), "> %s", pathNames[i]);
        } else {
            // Regular display for non-selected paths
            snprintf(line, sizeof(line), "  %s", pathNames[i]);
        }
        
        // Display on controller screen (3 lines available, shows 3 at a time)
        // Row 0, 1, 2 available
        if (i < 3) {
            master.set_text(i, 0, line);
        }
    }
    
    // If selection is in bottom 3, show those instead
    if (currentSelection >= 3) {
        master.clear();
        for (int i = 3; i < 6; i++) {
            char line[20];
            if (i == currentSelection) {
                snprintf(line, sizeof(line), "> %s", pathNames[i]);
            } else {
                snprintf(line, sizeof(line), "  %s", pathNames[i]);
            }
            master.set_text(i - 3, 0, line);
        }
    }
}

int selectAutonomousPath() {
    int selectedPath = 0;  // Current selection (0-5 for paths 1-6)
    bool confirmed = false;
    
    // Track button states to prevent multiple triggers
    bool upPressed = false;
    bool downPressed = false;
    bool aPressed = false;
    
    // Display initial selection
    displayPathSelection(selectedPath);
    
    // Selection loop
    while (!confirmed) {
        // Read button states
        bool upButton = master.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
        bool downButton = master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
        bool aButton = master.get_digital(pros::E_CONTROLLER_DIGITAL_A);
        
        // Navigate up (with debounce)
        if (upButton && !upPressed) {
            selectedPath--;
            if (selectedPath < 0) {
                selectedPath = 5;  // Wrap to bottom
            }
            displayPathSelection(selectedPath);
            upPressed = true;
        } else if (!upButton) {
            upPressed = false;
        }
        
        // Navigate down (with debounce)
        if (downButton && !downPressed) {
            selectedPath++;
            if (selectedPath > 5) {
                selectedPath = 0;  // Wrap to top
            }
            displayPathSelection(selectedPath);
            downPressed = true;
        } else if (!downButton) {
            downPressed = false;
        }
        
        // Confirm selection with A button (with debounce)
        if (aButton && !aPressed) {
            confirmed = true;
            aPressed = true;
        } else if (!aButton) {
            aPressed = false;
        }
        
        // Small delay to prevent overwhelming the controller
        pros::delay(50);
    }
    
    // Clear controller and show confirmation
    master.clear();
    char confirmMsg[20];
    snprintf(confirmMsg, sizeof(confirmMsg), "Selected: %s", pathNames[selectedPath]);
    master.set_text(0, 0, confirmMsg);
    master.set_text(1, 0, "Ready!");
    
    // Don't touch brain screen - analytics is running there
    // The analytics display will show the selected autonomous
    
    // Return 1-6 (not 0-5)
    return selectedPath + 1;
}
