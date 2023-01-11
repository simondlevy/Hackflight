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

#include "utils.h"

class Led {

    private:

        typedef enum {
            LED_WARNING_OFF = 0,
            LED_WARNING_ON,
            LED_WARNING_FLASH
        } ledWarningVehicleState_e;

        bool m_ledOn;

        ledWarningVehicleState_e m_ledWarningVehicleState = LED_WARNING_OFF;

        uint32_t m_ledWarningTimer = 0;

        void ledToggle(void)
        {
            m_ledOn = !m_ledOn;
            set(m_ledOn);
        }

        void ledWarningRefresh(void)
        {
            switch (m_ledWarningVehicleState) {
                case LED_WARNING_OFF:
                    set(false);
                    break;
                case LED_WARNING_ON:
                    set(true);
                    break;
                case LED_WARNING_FLASH:
                    ledToggle();
                    break;
            }

            auto now = micros();
            m_ledWarningTimer = now + 500000;
        }

    public:

        uint8_t pin;
        bool inverted;

        void set(bool on)
        {
            if (pin > 0) {
                digitalWrite(pin, inverted ? on : !on);
            }

            m_ledOn = on;
        }

        void begin(void)
        {
            if (pin > 0) {
                pinMode(pin, OUTPUT);
            }
        }

        void flash(uint8_t reps, uint16_t delayMs)
        {
            set(false);
            for (auto i=0; i<reps; i++) {
                ledToggle();
                delay(delayMs);
            }
            set(false);
        }

        void warningFlash(void)
        {
            m_ledWarningVehicleState = LED_WARNING_FLASH;
        }

        void warningDisable(void)
        {
            m_ledWarningVehicleState = LED_WARNING_OFF;
        }

        void warningUpdate(void)
        {
            uint32_t now = micros();

            if ((int32_t)(now - m_ledWarningTimer) < 0) {
                return;
            }

            ledWarningRefresh();
        }

}; // class Led
