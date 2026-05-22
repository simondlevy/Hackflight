/*
   Hackflight main sketch for Teensy using ESP32 receiver with MSP protocol

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

void serialEvent1()
{
    static MspParser _parser;

    while (Serial1.available()) {

        _parser = MspParser::parse(_parser, Serial1.read());

        const auto msgid = MspParser::getid(_parser); 

        if (msgid) {
            printf("msgid=%d\n", msgid);
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

    // Run motor mixer and motors
    //_core.runMotors(_rxdata.is_armed, _stabilizerPid.setpoint);
}
