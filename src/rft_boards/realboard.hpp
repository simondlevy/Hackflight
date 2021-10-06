/*
   Board subclass for real (hardware) boards

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "../RFT_board.hpp"
#include "../HF_debugger.hpp"
#include "../HF_filters.hpp"

namespace rft {

    class RealBoard : public Board {

        friend class SerialTask;

        private:

            static constexpr float   LED_STARTUP_FLASH_SECONDS = 1.0;
            static constexpr uint8_t LED_STARTUP_FLASH_COUNT   = 20;
            static constexpr float   LED_SLOWFLASH_SECONDS     = 0.25;

            bool _shouldFlash = false;

        protected:

            static const uint32_t SERIAL_BAUD = 115200;

            void begin(void)
            {
                // Flash LED
                float pauseSeconds = LED_STARTUP_FLASH_SECONDS /
                                     LED_STARTUP_FLASH_COUNT;
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
                    hf::Debugger::printf("%s\n", errmsg);
                    delaySeconds(0.1);
                }
            }

            virtual void setLed(bool isOn) = 0;

            virtual uint8_t serialAvailable(bool secondaryPort) = 0;

            virtual uint8_t serialRead(bool secondaryPort) = 0;

            virtual void serialWrite(uint8_t c, bool secondaryPort) = 0;

    }; // class RealBoard

} // namespace rft
