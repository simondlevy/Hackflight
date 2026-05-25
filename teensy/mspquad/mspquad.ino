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

#include <hackflight.h>
#include <firmware/debugging.hpp>
#include <firmware/quadcore.hpp>
#include <firmware/msp/parser.hpp>
using namespace hf;

static QuadCore _quadcore;

static msp_message_t _message;

void serialEvent1()
{
    static MspParser _parser;

    while (Serial1.available()) {

        _parser = MspParser::parse(_parser, Serial1.read());

        switch (MspParser::getid(_parser)) {

            case MSP_SET_ARMING:
                _message.is_armed = !_message.is_armed;
                _message.timestamp_msec = millis();
                break;

            case MSP_SET_IDLE:
                _message.is_hovering = false;
                _message.timestamp_msec = millis();
                break;

            case MSP_SET_HOVER:
                _message.is_hovering = true;
                _message.setpoint.thrust = MspParser::getFloat(_parser, 0);
                _message.setpoint.pitch = MspParser::getFloat(_parser, 1); // vx
                _message.setpoint.roll = MspParser::getFloat(_parser, 2); // vy
                _message.setpoint.yaw = MspParser::getFloat(_parser, 3);
                _message.timestamp_msec = millis();
                break;

            default:
                break;
        }
    }
}

void setup()
{
    // Start core sensors and motors (this will also start Serial1)
    _quadcore.begin();
}

void loop()
{
    // Run core algorithm to get setpoint from PID controllers
    const auto setpoint = _quadcore.update(_message);

    // Run motor mixer on setpoint
    _quadcore.runMotors(setpoint);
}
