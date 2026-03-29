/*
   Hackflight main sketch

   Copyright (C) 2026 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

// Third-party libraries
#include <dshot-teensy4.hpp>  

// Hackflight library
#include <hackflight.h>
#include <firmware/core.hpp>
#include <firmware/led.hpp>
#include <firmware/rx/elrs.hpp>
#include <firmware/sensors/imu.hpp>

static auto _rx = hf::RX(&Serial5);

static DshotTeensy4 _motors = DshotTeensy4({2, 3, 4, 5});

static hf::LED _led = hf::LED(13);

static hf::IMU _imu;

static hf::Core _core;

void setup()
{
    _core.setup(_imu, _rx, _motors, _led);
}

void loop()
{
    _core.loop(_imu, _rx, _motors, _led); 
}
