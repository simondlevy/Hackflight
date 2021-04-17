/*
   CPPM receiver support for Arduino-based flight controllers

   Copyright (c) 2018 Simon D. Levy

   MIT License
 */

#pragma once

#include "receiver.hpp"
#include <CPPMRX.h>

namespace hf {

    class CPPM_Receiver : public Receiver {

        private:

            static const uint16_t PPM_MIN = 990;
            static const uint16_t PPM_MAX = 2020;

            CPPMRX * rx = NULL;

        protected:

            void begin(void)
            {
                rx->begin();
            }

            bool gotNewFrame(void)
            {
                return rx->gotNewFrame();
            }

            void readRawvals(void)
            {
                uint16_t rcData[6];

                rx->computeRC(rcData);

                for (uint8_t k=0; k<6; k++) {

                    rawvals[k] = 2.f * (rcData[k] - PPM_MIN) / (PPM_MAX - PPM_MIN) - 1;
                }
            }

            bool lostSignal(void)
            {
                return false;
            }

        public:

            CPPM_Receiver(uint8_t pin, const uint8_t channelMap[6], const float demandScale) 
                : Receiver(channelMap, demandScale) 
            { 
                rx = new CPPMRX(pin, 6);
            }

    }; // class CPPM_Receiver

} // namespace hf
