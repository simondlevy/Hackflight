/*
   Hackflight main sketch for Teensy quadcopter using ESP32 receiver
   with MSP protocol

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

// Hackflight library
#include <hackflight.h>
#include <firmware/fc.hpp>
#include <firmware/effectors/quad_dshot.hpp>

static hf::FlightController _fc;

static hf::GamepadReceiver _rx;

static hf::QuadDshot _effector;

void serialEvent1()
{
    _rx.HandleSerial1Event();
}

void setup()
{
    // Start core devices
    _fc.Begin();

    // Start motors
    _effector.Begin();
}

void loop()
{
    // Run core algorithm to get setpoint from PID controllers
    const auto setpoint = _fc.Update(_rx, _effector.GetMotorValues(), 4);

    // Run the mixer and motors
    _effector.Run(_fc, setpoint);
}
