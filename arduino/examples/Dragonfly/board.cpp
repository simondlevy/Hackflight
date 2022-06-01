/*
This file is part of Hackflight.

Hackflight is free software: you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
Hackflight. If not, see <https://www.gnu.org/licenses/>.
*/

#include <Arduino.h>
#include <Wire.h>

#include <stdbool.h>
#include <stdint.h>

#include "rx/sbus.h"
#include "serial.h"
#include "time.h"

static const uint8_t LED_PIN = 13;

extern "C" {

void delayMillis(timeMs_t ms)
{
    delay(ms);
}

timeMs_t timeMillis(void)
{
    return millis();
}

timeUs_t timeMicros(void)
{
    return micros();
}

void boardInit(void)
{
    Serial.begin(115200);

    sbusInit(SERIAL_PORT_USART1);

    Wire.begin();

    delay(100);
}

static bool _ledOn;

void ledInit(void)
{
    pinMode(LED_PIN, OUTPUT);
}

void ledSet(bool on)
{
    digitalWrite(LED_PIN, on);
    _ledOn = on;
}

void ledToggle(void)
{
    _ledOn = !_ledOn;
    digitalWrite(LED_PIN, _ledOn);
}

uint32_t systemClockMicrosToCycles(uint32_t micros)
{
    return micros * clockCyclesPerMicrosecond();
}

uint32_t systemGetCycleCounter(void)
{
    return SystemCoreClock;
}

void systemReboot(void)
{
    // unused
}

} // extern "C"
