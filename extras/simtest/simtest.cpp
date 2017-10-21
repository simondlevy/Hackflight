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

#include <math.h>
#include <time.h>

#include <hackflight.hpp>
#include <models/3dfly.hpp> // arbitrary
#include <receivers/sim.hpp>

namespace hf {

class NullBoard : public Board {

    virtual void init(Config& config) override
    {
        (void)config;
    }

    virtual void delayMilliseconds(uint32_t msec) override
    {
    }

    virtual void getImu(float eulerAnglesRadians[3], int16_t gyroRaw[3]) override
    {
        for (int k=0; k<3; ++k) {
            eulerAnglesRadians[k] = 0;
            gyroRaw[k] = 0;
        }
    }

    virtual uint64_t getMicros() override
    {
        struct timespec t;
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t);
        return 1000000*t.tv_sec + t.tv_nsec/1000;
    }

    virtual void writeMotor(uint8_t index, float value) override
    {
        (void)index;
        (void)value;
    }


}; // class NullBoard

} // namespace hf

int main(int argc, char ** argv)
{
    hf::Hackflight h;
    hf::NullBoard  board;
    hf::Controller controller;
    hf::ThreeDFly  model;

    h.init(&board, &controller, &model);

    while (true) {
        h.update();
    }

    return 0;
}
