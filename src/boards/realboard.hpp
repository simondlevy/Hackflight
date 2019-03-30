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

// Support delay(), micros() on non-Arduino boards
#ifndef ARDUINO
extern "C" {
    extern uint32_t micros(void);
    extern void     delay(uint32_t msec);
}
#endif

namespace hf {

    class RealBoard : public Board {

        friend class MspSensor;

        private:

            static constexpr float   LED_STARTUP_FLASH_SECONDS = 1.0;
            static constexpr uint8_t LED_STARTUP_FLASH_COUNT   = 20;
            static constexpr float   LED_SLOWFLASH_SECONDS     = 0.25;

            bool _shouldFlash = false;

            // Supports MSP over wireless protcols like Bluetooth
            bool _useSerialTelemetry = false;

        protected:

            virtual void setLed(bool isOn) = 0;

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

                _shouldFlash = false;
            }

            float getTime(void)
            {
                return micros() / 1.e6f;
            }

            void delaySeconds(float sec)
            {
                delay((uint32_t)(1000*sec));
            }

            uint8_t serialAvailableBytes(void)
            {
                // Attempt to use telemetry first
                if (serialTelemetryAvailable()) {
                    _useSerialTelemetry = true;
                    return serialTelemetryAvailable();
                }

                // Default to USB
                if (serialNormalAvailable() > 0) {
                    _useSerialTelemetry = false;
                    return serialNormalAvailable();
                }

                return 0;
            }

            uint8_t serialReadByte(void)
            {
                return _useSerialTelemetry? serialTelemetryRead() : serialNormalRead();
            }

            void serialWriteByte(uint8_t c)
            {
                if (_useSerialTelemetry) {
                    serialTelemetryWrite(c);
                }
                else {
                    serialNormalWrite(c);
                }
            }

            virtual uint8_t serialNormalAvailable(void) = 0;

            virtual uint8_t serialNormalRead(void) = 0;

            virtual void    serialNormalWrite(uint8_t c) = 0;

            virtual uint8_t serialTelemetryAvailable(void)
            {
                return 0;
            }

            virtual uint8_t serialTelemetryRead(void)
            {
                return 0;
            }

            virtual void serialTelemetryWrite(uint8_t c)
            {
                (void)c;
            }

            void showArmedStatus(bool armed)
            {
                // Set LED to indicate armed
                if (!_shouldFlash) {
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

                _shouldFlash = shouldflash;
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
