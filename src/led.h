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

    friend class Arming;
    friend class Hackflight;

    private:

        typedef enum {
            WARNING_LED_OFF = 0,
            WARNING_LED_ON,
            WARNING_LED_FLASH
        } warningLedVehicleState_e;

        warningLedVehicleState_e m_warningLedVehicleState = WARNING_LED_OFF;

        uint32_t m_warningLedTimer = 0;

        void warningRefresh(void)
        {
            switch (m_warningLedVehicleState) {
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

            auto now = timeMicros();
            m_warningLedTimer = now + 500000;
        }

        void begin(void)
        {
            devInit();
        }

        void flash(uint8_t reps, uint16_t delayMs)
        {
            devSet(false);
            for (auto i=0; i<reps; i++) {
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
            m_warningLedVehicleState = WARNING_LED_OFF;
        }

        void warningFlash(void)
        {
            m_warningLedVehicleState = WARNING_LED_FLASH;
        }

        void warningUpdate(void)
        {
            uint32_t now = timeMicros();

            if ((int32_t)(now - m_warningLedTimer) < 0) {
                return;
            }

            warningRefresh();
        }
    protected:

        uint8_t m_pin = 0;

        Led(uint8_t pin)
        {
            m_pin = pin;
        }

        virtual void devInit(void) = 0;

        virtual void devSet(bool on) = 0;

        virtual void devToggle(void) = 0;
};
