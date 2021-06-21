/*
   Futaba SBUS receiver support for Arduino flight controllers

   Uses SBUS library from https://github.com/bolderflight/SBUS

   Copyright (C) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "receiver.hpp"
#include <SBUS.h>

namespace hf {

    class SBUS_Receiver : public Receiver {

        private:

            const uint16_t MAX_FAILSAFE = 10;

            SBUS rx = SBUS(Serial1);

            // These values must persist between calls to readRawvals()
            float    _channels[16];
            uint16_t _failsafeCount;

        protected:

            void begin(void)
            {
                rx.begin();
            }

            bool gotNewFrame(void)
            {
                bool failsafe = false;
                bool lostFrame = false;
                if (rx.readCal(_channels, &failsafe, &lostFrame)) {

                    // accumulate consecutive failsafe hits
                    if (failsafe) {
                        _failsafeCount++;
                    }
                    else { // reset count
                        _failsafeCount = 0;
                    }

                    return true;
                }

                return false;
            }

            void readRawvals(void)
            {
                memset(rawvals, 0, MAXCHAN*sizeof(float));
                memcpy(rawvals, _channels, MAXCHAN*sizeof(float));
            }

            bool lostSignal(void)
            {
                return _failsafeCount > MAX_FAILSAFE;
            }

        public:

            SBUS_Receiver(
                    const uint8_t channelMap[6], 
                    const float demandScale)
                :  Receiver(channelMap, demandScale) 
            { 
                _failsafeCount = 0;
            }

    }; // class SBUS_Receiver

} // namespace hf
