/*
   Copyright (c) 2022 Simon D. Levy

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

#include "led.h"
#include "time.h"

typedef enum {
    WARNING_LED_OFF = 0,
    WARNING_LED_ON,
    WARNING_LED_FLASH
} warningLedState_e;

static warningLedState_e warningLedState = WARNING_LED_OFF;
static uint32_t warningLedTimer = 0;

static void warningLedRefresh(void)
{
    switch (warningLedState) {
        case WARNING_LED_OFF:
            ledDevSet(false);
            break;
        case WARNING_LED_ON:
            ledDevSet(true);
            break;
        case WARNING_LED_FLASH:
            ledDevToggle();
            break;
    }

    uint32_t now = timeMicros();
    warningLedTimer = now + 500000;
}

// ----------------------------------------------------------------------------

void ledFlash(uint8_t reps, uint16_t delayMs)
{
    ledDevSet(false);
    for (uint8_t i=0; i<reps; i++) {
        ledDevToggle();
        delayMillis(delayMs);
    }
    ledDevSet(false);
}


void ledWarningDisable(void)
{
    warningLedState = WARNING_LED_OFF;
}

void ledWarningFlash(void)
{
    warningLedState = WARNING_LED_FLASH;
}

void ledWarningUpdate(void)
{
    uint32_t now = timeMicros();

    if ((int32_t)(now - warningLedTimer) < 0) {
        return;
    }

    warningLedRefresh();
}


