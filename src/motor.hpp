/*
   Abstract parent class for running motors on Arduino

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#ifdef ESP32
#include <analogWrite.h>
#endif

namespace hf {

    class Motor {

        protected:

            static const uint8_t MAX_COUNT = 20; // arbitrary

            uint8_t _pins[MAX_COUNT];
            uint8_t _count = 0;

            Motor(const uint8_t count) 
            {
                _count = count;
            }

            Motor(const uint8_t * pins, const uint8_t count)
            {
                for (uint8_t k=0; k<count; ++k) {
                    _pins[k] = pins[k];
                }
                _count = count;
            }

        public:

            virtual void begin(void) { }

            virtual void write(uint8_t index, float value) = 0;

    }; // class Motor

} // namespace hf
