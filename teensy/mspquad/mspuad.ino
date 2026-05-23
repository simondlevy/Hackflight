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
#include <firmware/quadcore.hpp>
#include <firmware/msp/parser.hpp>
using namespace hf;

static QuadCore _core;

typedef struct {

    uint32_t timestamp;
    bool armed;
    bool hovering;
    Setpoint setpoint;

} message_t;

static message_t _message;

void serialEvent1()
{
    static MspParser _parser;

    while (Serial1.available()) {

        _parser = MspParser::parse(_parser, Serial1.read());

        switch (MspParser::getid(_parser)) {

            case MSP_SET_ARMING:
                _message.armed = !_message.armed;
                _message.timestamp = millis();
                break;

            case MSP_SET_IDLE:
                _message.hovering = false;
                _message.timestamp = millis();
                break;

            case MSP_SET_HOVER:
                _message.hovering = true;
                _message.setpoint.thrust = MspParser::getFloat(_parser, 0);
                _message.setpoint.pitch = MspParser::getFloat(_parser, 1); // vx
                _message.setpoint.roll = MspParser::getFloat(_parser, 2); // vy
                _message.setpoint.yaw = MspParser::getFloat(_parser, 3);
                _message.timestamp = millis();
                break;

            default:
                break;
        }
    }
}

void setup()
{
    // Start core sensors and motors (this will also start Serial1)
    _core.begin();
}

void loop()
{
    // Read core sensors and do sensor fusion
    _core.getState();

    printf("armed=%d hovering=%d\n", _message.armed, _message.hovering);

    // Run motor mixer and motors
    //_core.runMotors(_rxdata.is_armed, _stabilizerPid.setpoint);
}
