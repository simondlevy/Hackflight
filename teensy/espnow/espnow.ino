/*
   Hackflight with ESPNOW receiver

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
#include <firmware/debugging.hpp>
#include <firmware/led.hpp>
#include <firmware/msp/parser.hpp>
#include <firmware/rx.hpp>
using namespace hf;

static const uint8_t LED_PIN = LED_BUILTIN;

static auto _led = LED(LED_PIN);
static Debugger _debugger;
static MspParser _parser;
static RX _rx;

void serialEvent1()
{
    while (Serial1.available()) {

        _parser = MspParser::parse(_parser, Serial1.read());

        if (MspParser::getid(_parser) == 203) {

            _rx = RX::update(
                    _rx, 
                    MspParser::getUshort(_parser, 0),
                    MspParser::getUshort(_parser, 1),
                    MspParser::getUshort(_parser, 2),
                    MspParser::getUshort(_parser, 3),
                    MspParser::getUshort(_parser, 4),
                    millis());
        }
    }
}

// Setup
void setup()
{
    Serial1.begin(115200);
    _led.begin(); 
}

// Loop
void loop()
{
    _led.blink(true);

    _rx = RX::checkTimeout(_rx, millis());

    _debugger.report(_rx);
}
