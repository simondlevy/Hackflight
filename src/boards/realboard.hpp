/*
   realboard.hpp : Board subclass for real (hardware) boards

   Copyright (c) 2018 Simon D. Levy

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
#include "debug.hpp"
#include "datatypes.hpp"

namespace hf {

    class RealBoard : public Board {

        private:

            static constexpr float   LED_STARTUP_FLASH_SECONDS = 1.0;
            static constexpr uint8_t LED_STARTUP_FLASH_COUNT   = 20;
            static constexpr float   LED_SLOWFLASH_SECONDS     = 0.25;

            bool _shouldflash;

        protected:

            virtual uint32_t getMicroseconds(void) = 0;
            virtual void     delaySeconds(float time) = 0;
            virtual void     setLed(bool is_on) = 0;

            void init(void)
            {
                // Flash LED
                float pauseSeconds = LED_STARTUP_FLASH_SECONDS / LED_STARTUP_FLASH_COUNT;
                setLed(false);
                for (uint8_t i = 0; i < LED_STARTUP_FLASH_COUNT; i++) {
                    setLed(true);
                    delaySeconds(pauseSeconds);
                    setLed(false);
                    delaySeconds(pauseSeconds);
                }
                setLed(false);

                _shouldflash = false;
            }

            float getTime(void)
            {
                return getMicroseconds() / 1.e6f;
            }

            void showArmedStatus(bool armed)
            {
                // Set LED to indicate armed
                if (!_shouldflash) {
                    setLed(armed);
                }
            }

            void flashLed(bool shouldflash)
            {
                if (shouldflash) {

                    static float _time;
                    static bool state;

                    float time = getTime();

                    if (time-_time > LED_SLOWFLASH_SECONDS) {
                        state = !state;
                        setLed(state);
                        _time = time;
                    }
                }

                _shouldflash = shouldflash;
            }

            void error(const char * errmsg) 
            {
                while (true) {
                    Debug::printf("%s\n", errmsg);
                    delaySeconds(0.1);
                }
            }

 
    }; // class RealBoard

} // namespace hf
