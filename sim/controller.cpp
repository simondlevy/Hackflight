/*
   controller.cpp : support for various kinds of flight-simulator control devices

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
#include <linux/joystick.h>


// AxialController ----------------------------------------------------------------------

void Controller::getDemands(
        float & pitchDemand, 
        float & rollDemand, 
        float & yawDemand, 
        float & throttleDemand,
        float & auxDemand)
{
    this->update();

    pitchDemand    = this->pitch;
    rollDemand     = this->roll;
    yawDemand      = this->yaw;
    throttleDemand = this->throttle;
    auxDemand      = this->aux;
}

// AxialController ----------------------------------------------------------------------

void AxialController::init(const char * devname)
{
    this->joyfd = open(devname, O_RDONLY);

    if(this->joyfd > 0) 
        fcntl(this->joyfd, F_SETFL, O_NONBLOCK);
}

void AxialController::update(void)
{
    if (this->joyfd > 0) {

        struct js_event js;

        read(joyfd, &js, sizeof(struct js_event));

        if (js.type & JS_EVENT_AXIS) 
            this->js2demands(js.number, js.value / 32767.);

        if (js.type & JS_EVENT_BUTTON) 
            this->handle_button(js.number);
   }

   this->postprocess();
}

void AxialController::stop(void)
{
    if (this->joyfd > 0) 
        close(this->joyfd);
}


// PS3Controller ---------------------------------------------------------------------------

void PS3Controller::init(void)
{
    AxialController::init();

    this->throttle = 0;
    this->timeprev = 0;
    this->timecount = 0;
    this->aux = +1;
    this->ready = false;
}

void PS3Controller::js2demands(int jsnumber, float jsvalue) 
{
    switch (jsnumber) {
        case 0:
            this->yaw = -jsvalue;
            break;
        case 1: 
            this->throttleDirection = jsvalue < 0 ? +1 : (jsvalue > 0 ? -1 : 0);
            break;
        case 2:
            this->roll = -jsvalue;
            break;
        case 3:
            this->pitch = jsvalue;
    }
}

void PS3Controller::postprocess(void)
{
    struct timeval tv;
    gettimeofday(&tv,NULL);

    double timecurr = tv.tv_sec + tv.tv_usec/1e6;

    if (this->timeprev) {
        this->timeavg += (timecurr-this->timeprev);
    }

    this->timeprev = timecurr; 
    this->timecount++;

    this->throttle += this->throttleDirection * this->timeavg / this->timecount / 10;

    if (this->throttle < 0)
        this->throttle = 0;
    if (this->throttle > 1)
        this->throttle = 1;

}

void PS3Controller::handle_button(int number)
{
    // Ignore button registration on startup
    if (number == 11 && !ready)
        ready = true;

    if (number < 4 && ready)
        switch (number) {
            case 0:
                this->aux = +1;
                break;
            case 1:
                this->aux = -1;
                break;
            case 3:
                this->aux = 0;
        }
}

// TaranisController ---------------------------------------------------------------------------

void TaranisController::js2demands(int jsnumber, float jsvalue) 
{
    // Helps avoid bogus throttle values at startup
    static int throttleCount;

    switch (jsnumber) {

        case 0:
            if (throttleCount > 2)
                this->throttle = (jsvalue + 1) / 2;
            throttleCount++;
            break;

        case 1:
            this->roll = -jsvalue;
            break;

        case 2:
            this->pitch = jsvalue;
            break;

        case 3:
            this->yaw = -jsvalue;
            break;

        case 5:
            this->aux = -jsvalue;
    }
}

void TaranisController::postprocess(void)
{
}

void TaranisController::handle_button(int number)
{
}

// KeyboardController ----------------------------------------------------------------------

KeyboardController::KeyboardController(void) {
}

void KeyboardController::init(void) {

    struct termios newSettings;

    // Save keyboard settings for restoration later
    tcgetattr(fileno( stdin ), &this->oldSettings);

    // Create new keyboard settings
    newSettings = this->oldSettings;
    newSettings.c_lflag &= (~ICANON & ~ECHO);
    tcsetattr(fileno(stdin), TCSANOW, &newSettings);

    this->pitch    = 0;
    this->roll     = 0;
    this->yaw      = 0;
    this->throttle = 0;
    this->aux = +1;
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

void KeyboardController::update(void)
{
    fd_set set;
    struct timeval tv;

    tv.tv_sec = 0;
    tv.tv_usec = 1000;

    FD_ZERO( &set );
    FD_SET( fileno( stdin ), &set );

    int res = select( fileno( stdin )+1, &set, NULL, NULL, &tv );

    if( res > 0 )
    {
        char c;
        read(fileno( stdin ), &c, 1);
        switch (c) {
            case 10:
                full_decrement(&this->yaw);
                break;
            case 50:
                full_increment(&this->yaw);
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
                full_decrement(&this->roll);
                break;
            case 68:
                full_increment(&this->roll);
                break;
            case 47:
                this->aux = +1;
                break;
            case 42:
                this->aux = 0;
                break;
            case 45:
                this->aux = -1;
                break;
        }
    }
    else if( res < 0 ) {
        perror( "select error" );
    }
}


void KeyboardController::stop(void)
{
    tcsetattr(fileno( stdin ), TCSANOW, &this->oldSettings);
}
