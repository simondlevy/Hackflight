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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "led_device.h"
#include "time.h"

void ledFlash(uint8_t reps, uint16_t delayMs);
void ledWarningDisable(void);
void ledWarningFlash(void);
void ledWarningUpdate(void);

class LED {

    private:

        typedef enum {
            WARNING_LED_OFF = 0,
            WARNING_LED_ON,
            WARNING_LED_FLASH
        } warningLedState_e;

        warningLedState_e m_warningLedState = WARNING_LED_OFF;

        uint32_t m_warningLedTimer = 0;

        void warningRefresh(void)
        {
            switch (m_warningLedState) {
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
            m_warningLedTimer = now + 500000;
        }

    public:

        void flash(uint8_t reps, uint16_t delayMs)
        {
            ledDevSet(false);
            for (uint8_t i=0; i<reps; i++) {
                ledDevToggle();
                delayMillis(delayMs);
            }
            ledDevSet(false);
        }

        void warningDisable(void)
        {
            m_warningLedState = WARNING_LED_OFF;
        }

        void warningFlash(void)
        {
            m_warningLedState = WARNING_LED_FLASH;
        }

        void warningUpdate(void)
        {
            uint32_t now = timeMicros();

            if ((int32_t)(now - m_warningLedTimer) < 0) {
                return;
            }

            warningRefresh();
        }
};

