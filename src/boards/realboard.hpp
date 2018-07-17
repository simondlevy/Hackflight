/*
   realboard.hpp : Board subclass for real (hardware) boards

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "board.hpp"
#include "datatypes.hpp"

namespace hf {

    class RealBoard : public Board {

        private:

            const uint32_t LED_STARTUP_FLASH_MILLI = 1000;
            const uint32_t LED_STARTUP_FLASH_COUNT = 20;
            const uint32_t LED_SLOWFLASH_MICROS    = 250000;

            bool _shouldflash;

        protected:

            virtual void delayMilliseconds(uint32_t msec) { (void)msec; } 
            virtual void ledSet(bool is_on) { (void)is_on; }

            void init(void)
            {
                // Flash LED
                uint32_t pauseMilli = LED_STARTUP_FLASH_MILLI / LED_STARTUP_FLASH_COUNT;
                ledSet(false);
                for (uint8_t i = 0; i < LED_STARTUP_FLASH_COUNT; i++) {
                    ledSet(true);
                    delayMilliseconds(pauseMilli);
                    ledSet(false);
                    delayMilliseconds(pauseMilli);
                }
                ledSet(false);

                _shouldflash = false;
            }

            void showArmedStatus(bool armed)
            {
                // Set LED to indicate armed
                if (!_shouldflash) {
                    ledSet(armed);
                }
            }

            void flashLed(bool shouldflash)
            {
                if (shouldflash) {

                    static uint32_t _usec;
                    static bool state;

                    uint32_t usec = getMicroseconds();

                    if (usec-_usec > LED_SLOWFLASH_MICROS) {
                        state = !state;
                        ledSet(state);
                        _usec = usec;
                    }
                }

                _shouldflash = shouldflash;
            }

    }; // class RealBoard

} // namespace hf
