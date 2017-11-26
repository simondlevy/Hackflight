/*
   simtest.cpp : Simple simulator for Hackflight, to support testing new controllers

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

#include <hackflight.hpp>
#include <models/3dfly.hpp> // arbitrary
#include <receivers/sim.hpp>
#include <boards/sim.hpp>

#include <time.h>
#include <stdio.h>

static const float DELTA_SEC = .001;

static float getsec(void)
{
    struct timespec t;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t);
    return t.tv_sec + t.tv_nsec/1.e9;
}

static void delay(float dsec)
{
    float startsec = getsec();
    while (getsec()-startsec < dsec)
        ;
}

int main(int argc, char ** argv)
{
    hf::Hackflight h;
    hf::SimBoard  board(DELTA_SEC);
    hf::Controller controller;
    hf::ThreeDFly  model;

    h.init(&board, &controller, &model);

    while (true) {

        delay(DELTA_SEC);

        h.update();
    }

    return 0;
}
