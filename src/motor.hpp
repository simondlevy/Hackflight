/*
   Abstract class for motors

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once


namespace hf {

    class Motor {

        protected:

            uint8_t _pin = 0;

            Motor(uint8_t pin)
            {
                _pin = pin;
            }

        public:

            virtual void begin(void) { }

            virtual void write(float value) = 0;

    }; // class Motor

} // namespace hf
