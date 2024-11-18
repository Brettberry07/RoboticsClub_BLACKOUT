#pragma once
#include "globals.hpp"

struct Button{
    int x;
    int y;
    int width;
    int height;
    pros::Color color;
    bool isPressed;
    const char title;
    const char* imagePath;
};

extern Button buttons[];

void drawButton(Button& button);
bool buttonTouched(Button& button, int touchX, int touchY);