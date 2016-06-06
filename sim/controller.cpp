/*
   controller.cpp : support for various kinds of control devices

   Copyright (C) Simon D. Levy 2016

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.


   Adapted from  http://www.cplusplus.com/forum/general/5304/
*/

#include "controller.hpp"

KeyboardController::KeyboardController(void) {
}

KeyboardController::~KeyboardController(void) {

    tcsetattr( fileno( stdin ), TCSANOW, &this->oldSettings );
}

void KeyboardController::init(void) {

    struct termios newSettings;

    tcgetattr(fileno( stdin ), &this->oldSettings);
    newSettings = this->oldSettings;
    newSettings.c_lflag &= (~ICANON & ~ECHO);
    tcsetattr(fileno( stdin ), TCSANOW, &newSettings);

    this->pitch    = 0;
    this->roll     = 0;
    this->yaw      = 0;
    this->throttle = 0;
}

void KeyboardController::full_increment(float * value) 
{
    change(value, +1, -1, +1);
}

void KeyboardController::full_decrement(float * value) 
{
    change(value, -1, -1, +1);
}

void KeyboardController::pos_increment(float * value) 
{
    change(value, +1, 0, +1);
}

void KeyboardController::pos_decrement(float * value) 
{
    change(value, -1, 0, +1);
}

void KeyboardController::change(float *value, int dir, float min, float max) 
{
    *value += dir * INCREMENT;

    if (*value > max)
        *value = max;

    if (*value < min)
        *value = min;
}

void KeyboardController::getDemands(float & pitchDemand, float & rollDemand, float & yawDemand, float & throttleDemand) {
    pitchDemand = 0;
    rollDemand = 0;
    yawDemand = 0;
    throttleDemand = 0;

    fd_set set;
    struct timeval tv;

    tv.tv_sec = 1;
    tv.tv_usec = 0;

    FD_ZERO( &set );
    FD_SET( fileno( stdin ), &set );

    int res = select( fileno( stdin )+1, &set, NULL, NULL, &tv );

    if( res > 0 )
    {
        char c;
        read(fileno( stdin ), &c, 1);
        switch (c) {
            case 10:
                full_increment(&this->yaw);
                break;
            case 50:
                full_decrement(&this->yaw);
                break;
            case 53:
                pos_increment(&this->throttle);
                break;
            case 54:
                pos_decrement(&this->throttle);
                break;
            case 65:
                full_decrement(&this->pitch);
                break;
            case 66:
                full_increment(&this->pitch);
                break;
            case 67:
                full_increment(&this->roll);
                break;
            case 68:
                full_decrement(&this->roll);
                break;
        }
    }
    else if( res < 0 ) {
        perror( "select error" );
    }

    pitchDemand    = this->pitch;
    rollDemand     = this->roll;
    yawDemand      = this->yaw;
    throttleDemand = this->throttle;
}

int main()
{
    KeyboardController kb;

    kb.init();

    while ( 1 )
    {
        float pitchDemand, rollDemand, yawDemand, throttleDemand;

        kb.getDemands(pitchDemand, rollDemand, yawDemand, throttleDemand);

        printf("p: %+3.3f  | r: %+3.3f | y: %+3.3f  | t: %3.3f\n", 
                pitchDemand, rollDemand, yawDemand, throttleDemand);
    }

    return 0;
}
