/*
   Hackflight main sketch for Teensy quadcopter using ESP32 receiver with MSP
   protocol

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
#include <mixers/bfquadx.hpp>
using namespace hf;

static FC _fc;

static Mixer _mixer;

static DshotTeensy4 _motors = DshotTeensy4({2, 3, 4, 5});

void serialEvent1()
{
    _fc.handleSerial1Event();
}

void setup()
{
    // Start core devices
    _fc.beginFC();

    // Start hover-deck
    _fc.beginHoverDeck();

    // Start motors
    _motors.begin();
}

void loop()
{
    // Run core algorithm to get setpoint from PID controllers
    const auto setpoint = _fc.updateFCAndHover(_mixer.motorvals, 4);

    // Run motor mixer on setpoint
    _mixer = Mixer::run(_mixer, setpoint);

    const float nospin[4] = {0, 0, 0, 0};

    // Run motors if safe
    if (_fc.isSafeToFly()) {
        _motors.run(_fc.isArmed(), nospin); //_mixer.motorvals);
    }
}
