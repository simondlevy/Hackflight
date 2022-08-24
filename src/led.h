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

#include "time.h"

class Led {

    private:

        typedef enum {
            WARNING_LED_OFF = 0,
            WARNING_LED_ON,
            WARNING_LED_FLASH
        } warningLedState_e;

        warningLedState_e m_warningLedState = WARNING_LED_OFF;

        uint32_t m_warningLedTimer = 0;

        uint8_t m_pin = 0;

        void warningRefresh(void)
        {
            switch (m_warningLedState) {
                case WARNING_LED_OFF:
                    devSet(false);
                    break;
                case WARNING_LED_ON:
                    devSet(true);
                    break;
                case WARNING_LED_FLASH:
                    devToggle();
                    break;
            }

            uint32_t now = timeMicros();
            m_warningLedTimer = now + 500000;
        }

    protected:

        virtual void devInit(uint8_t pin) = 0;

        virtual void devSet(bool on) = 0;

        virtual void devToggle(void) = 0;
    public:

        Led(uint8_t pin)
        {
            m_pin = pin;
        }

        void begin(void)
        {
            devInit(m_pin);
        }

        void flash(uint8_t reps, uint16_t delayMs)
        {
            devSet(false);
            for (uint8_t i=0; i<reps; i++) {
                devToggle();
                delayMillis(delayMs);
            }
            devSet(false);
        }

        void set (bool onOff)
        {
            devSet(onOff);
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
