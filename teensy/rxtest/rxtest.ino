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

// Third-party libraries
#include <dshot-teensy4.hpp>  

// Hackflight library

#include <hackflight.h>
#include <datatypes.hpp>
#include <firmware/debugging.hpp>
#include <firmware/led.hpp>
#include <firmware/rx.hpp>
using namespace hf;

static const uint8_t LED_PIN = LED_BUILTIN;

static auto _led = LED(LED_PIN);

static Debugger _debugger;

void setup()
{
    RX::begin();
    _led.begin(); 
}

void loop()
{
    _led.blink(true);

    auto rxdata = RX::read();

    // Check receiver timeout
    rxdata = RX::Data::checkTimeout(rxdata, millis());

    _debugger.report(rxdata);

}
