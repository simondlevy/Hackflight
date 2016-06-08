/*
   controller.hpp : support for various kinds of flight-simulator control devices

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
*/

#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

class Controller {

    protected:

        Controller(void) { }

        float pitch;
        float roll;
        float yaw;
        float throttle;

        virtual void update(void) = 0;

    public:

        void getDemands(float & pitchDemand, float & rollDemand, float & yawDemand, float & throttleDemand);

        virtual void stop(void) = 0;
};

class AxialController : public Controller {

    public:
       
        void init(const char * devname="/dev/input/js0");

        void stop(void);

    protected:

        void update(void);

        virtual void js2demands(int jsnumber, float jsvalue) = 0;

        virtual void postprocess(void) = 0;

    private:

        int joyfd;
};

class TaranisController : public AxialController {

    protected:

        void js2demands(int jsnumber, float jsvalue);

        void postprocess(void);
};

class PS3Controller : public AxialController {

    private:

        double timeprev; 
        double timeavg; 
        unsigned long timecount;
        int throttleDirection;

    protected:

        void js2demands(int jsnumber, float jsvalue);

        void postprocess(void);

    public:

        void init(void);
};

/**
  *
  * Uses key bindings from <a href="http://www.flightsimbooks.com/flightsimhandbook/keyboardcontrols.php">
  * Microsoft Flight Simulator</a>. 
  */
class KeyboardController : public Controller {

    private:

        static const float INCREMENT = .01;

        struct termios oldSettings;

        static void full_increment(float * value);
        static void full_decrement(float * value);
        static void pos_increment(float * value); 
        static void pos_decrement(float * value); 
        static void change(float * value, int dir, float min, float max); 

    protected:

        void update(void);

    public:

        KeyboardController(void);

        ~KeyboardController(void);

        void init(void);

        void stop(void);
};

