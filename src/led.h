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
        } warningLedVehicleState_e;

        bool m_on;

        warningLedVehicleState_e m_warningLedVehicleState = WARNING_LED_OFF;

        uint32_t m_warningLedTimer = 0;

        void toggle(void)
        {
            m_on = !m_on;
            set(m_on);
        }

        void warningRefresh(void)
        {
            switch (m_warningLedVehicleState) {
                case WARNING_LED_OFF:
                    set(false);
                    break;
                case WARNING_LED_ON:
                    set(true);
                    break;
                case WARNING_LED_FLASH:
                    toggle();
                    break;
            }

            auto now = micros();
            m_warningLedTimer = now + 500000;
        }

    public:

        uint8_t pin;
        bool inverted;

        void set(bool on)
        {
            if (pin > 0) {
                digitalWrite(pin, inverted ? on : !on);
            }

            m_on = on;
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
                toggle();
                delay(delayMs);
            }
            set(false);
        }

        void warningFlash(void)
        {
            m_warningLedVehicleState = WARNING_LED_FLASH;
        }

        void warningDisable(void)
        {
            m_warningLedVehicleState = WARNING_LED_OFF;
        }

        void warningUpdate(void)
        {
            uint32_t now = micros();

            if ((int32_t)(now - m_warningLedTimer) < 0) {
                return;
            }

            warningRefresh();
        }

}; // class Led
