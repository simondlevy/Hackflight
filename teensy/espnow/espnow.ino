/*
   Hackflight for Teensy 4.0 with ELRs receiver

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
#include <firmware/rxdata.hpp>
using namespace hf;

static const uint8_t LED_PIN = LED_BUILTIN;

static auto _led = LED(LED_PIN);
static Debugger _debugger;
static MspParser _parser;

void serialEvent1()
{
    while (Serial1.available()) {

        _parser = MspParser::parse(_parser, Serial1.read());

        if (MspParser::getid(_parser) == 203) {

            static uint32_t _count;

            printf("%04lu: %04d %04d %04d %04d %04d\n",
                    ++_count,
                    MspParser::getUshort(_parser, 0),
                    MspParser::getUshort(_parser, 1),
                    MspParser::getUshort(_parser, 2),
                    MspParser::getUshort(_parser, 3),
                    MspParser::getUshort(_parser, 4));
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
    //_debugger.report(_rx.read());
}
