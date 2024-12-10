#include "globals.hpp"

/*
ScreenWidth = 480
ScreenHeight = 272
*/

const int BUTTON_WIDTH = 160;
const int BUTTON_HEIGHT = 136;

/*
PSUEDOCODE:
struct Button{
    int x, y, width, height;
    pros::Color color;
    bool isPressed;
    char buttonID;
    string imagePath;
}

void drawButton(button){
    if button is not pressed:
        set color to button color
    else:
        set color to gold
    fill rectangle with color
    draw image at button.x, button.y
}

bool buttonTouched(button, int touchX, int touchY){ //touchX and touchY are the coordinates of the touch
    if touchX is the buttons width:
        if touchY is within the buttons height:
            return true (button is pressed)
    return false (button is not pressed)
}
*/



Button buttons[] = {
    {0, 0, BUTTON_WIDTH, BUTTON_HEIGHT, pros::Color::red, false, '1', "icons/topLeft.c"},
    {160, 0, BUTTON_WIDTH, BUTTON_HEIGHT, pros::Color::green, false, '2', "icons/topRight.c"},
    {320, 0, BUTTON_WIDTH, BUTTON_HEIGHT, pros::Color::blue, false, '3', "icons/fieldAsset.c"},
    {0, 136, BUTTON_WIDTH, BUTTON_HEIGHT, pros::Color::yellow, false, '4', "icons/bottomLeft.c"},
    {160, 136, BUTTON_WIDTH, BUTTON_HEIGHT, pros::Color::purple, false, '5', "icons/bottomRight.c"},
    {320, 136, BUTTON_WIDTH, BUTTON_HEIGHT, pros::Color::pink, false, '6', "icons/fieldAsset.c"}
};

void drawButton(Button& button){
    // Set the image width and height to fill the area of the button
    // Created these just for readability (not efficiency)
    int image_width = button.width;
    int image_height = button.height;

    if (!button.isPressed){
        pros::screen::set_pen(button.color);
    }
    else{
        pros::screen::set_pen(pros::Color::gold);
    }
    pros::screen::fill_rect(button.x, button.y, button.x+button.width, button.y+button.height);

    // draw icons for the buttons
    // Not how this works hahaha
    // pros::screen::print(pros::E_TEXT_MEDIUM, button.x + (button.width / 2) - (image_width / 2), 
    //                     button.y + (button.height / 2) - (image_height / 2), 
    //                     button.imagePath);

    //Correctly 
    // Draw the image for the button
    LV_IMG_DECLARE(imagePath);
    lv_obj_t * buttonImage = lv_img_create(lv_scr_act());
    lv_img_set_src(buttonImage, &imagePath);
    lv_obj_set_pos(buttonImage, button.x, button.y);
    lv_obj_set_size(buttonImage, button.width, button.height);
}

// Checks for button presses
// Input:
// Button button: the button that is being checked
// int touchX: the x coordinate of the touch
// int touchY: the y coordinate of the touch
// Output:
// bool: returns true if the button is pressed, false if it is not
bool buttonTouched(Button& button, int touchX, int touchY){
    if( (touchX >= button.x && touchX <= button.x+button.width) &&
        (touchY >= button.y && touchY <= button.y+button.height )){
            return true;
        }
    return false;
}

