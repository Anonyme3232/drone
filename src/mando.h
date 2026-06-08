#pragma once

struct JoystickData {
    float roll;
    float pitch; 
};

void setupJoystick();
JoystickData readJoystick();