#include "globals.hpp"

/*
ScreenWidth = 480
ScreenHeight = 272
*/

const int BUTTON_WIDTH = 160;
const int BUTTON_HEIGHT = 136;



Button buttons[] = {
    {0, 0, BUTTON_WIDTH, BUTTON_HEIGHT, pros::Color::red, false, '1', "icons/bottomRight-Right.png"},
    {160, 0, BUTTON_WIDTH, BUTTON_HEIGHT, pros::Color::green, false, '2', "icons/bottomLeft-Left.png"},
    {320, 0, BUTTON_WIDTH, BUTTON_HEIGHT, pros::Color::blue, false, '3', "icons/fieldAsset.png"},
    {0, 136, BUTTON_WIDTH, BUTTON_HEIGHT, pros::Color::yellow, false, '4', "icons/bottomLeft-Left.png"},
    {160, 136, BUTTON_WIDTH, BUTTON_HEIGHT, pros::Color::purple, false, '5', "icons/bottomRight-Right.png"},
    {320, 136, BUTTON_WIDTH, BUTTON_HEIGHT, pros::Color::pink, false, '6', "icons/fieldAsset.png"}
};

void drawButton(Button& button){
    // Assumes tha the image is 1080x1080
    // The provided icon images are 1080x1080 but other images may not be
    int image_width = 1080;
    int image_height = 1080;
    if (!button.isPressed){
        pros::screen::set_pen(button.color);
    }
    else{
        pros::screen::set_pen(pros::Color::gold);
    }
    pros::screen::fill_rect(button.x, button.y, button.x+button.width, button.y+button.height);

    // Draws icon bc why not
    // TODO: Debug this
    pros::screen::print(pros::E_TEXT_MEDIUM, button.x + (button.width / 2) - (image_width / 2), 
                        button.y + (button.height / 2) - (image_height / 2), 
                        button.imagePath);
}

bool buttonTouched(Button& button, int touchX, int touchY){
    if( (touchX >= button.x && touchX <= button.x+button.width) &&
        (touchY >= button.y && touchY <= button.y+button.height )){
            return true;
        }
    return false;
}

