/*
   "Mock" receiver subclass for prototyping

   MIT License
 */

#pragma once

#include "receiver.hpp"

static constexpr uint8_t DEFAULT_MAP[6] = {0,1,2,3,4,5};
static constexpr float   DEFAULT_DEMAND_SCALE = 1.0f;

namespace hf {

    class MockReceiver : public Receiver {

        protected:

            void begin(void)
            {
            }

            virtual bool gotNewFrame(void) override
            {
                return false;
            }

            void readRawvals(void)
            {
            }

            bool lostSignal(void)
            {
                return false;
            }

        public:

            MockReceiver(void) 
                : Receiver(DEFAULT_MAP, DEFAULT_DEMAND_SCALE)
            { 
            }

    }; // class MockReceiver

} // namespace hf
