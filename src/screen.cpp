#include "globals.hpp"

/*
ScreenWidth = 480
ScreenHeight = 272
*/

const int BUTTON_WIDTH = 160;
const int BUTTON_HEIGHT = 136;



Button buttons[] = {
    {0, 0, BUTTON_WIDTH, BUTTON_HEIGHT, pros::Color::red, false, '1'},
    {160, 0, BUTTON_WIDTH, BUTTON_HEIGHT, pros::Color::green, false, '2'},
    {320, 0, BUTTON_WIDTH, BUTTON_HEIGHT, pros::Color::blue, false, '3'},
    {0, 136, BUTTON_WIDTH, BUTTON_HEIGHT, pros::Color::yellow, false, '4'},
    {160, 136, BUTTON_WIDTH, BUTTON_HEIGHT, pros::Color::purple, false, '5'},
    {320, 136, BUTTON_WIDTH, BUTTON_HEIGHT, pros::Color::pink, false, '6'}
};

void drawButton(Button& button){
    if (!button.isPressed){
        pros::screen::set_pen(button.color);
    }
    else{
        pros::screen::set_pen(pros::Color::gold);
    }
    pros::screen::fill_rect(button.x, button.y, button.x+button.width, button.y+button.height);
}

bool buttonTouched(Button& button, int touchX, int touchY){
    if( (touchX >= button.x && touchX <= button.x+button.width) &&
        (touchY >= button.y && touchY <= button.y+button.height )){
            return true;
        }
    return false;
}

