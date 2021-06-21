/*
   Abstract actuator class for mixers and receiver proxies

   Copyright (c) 2020 Simon D. Levy

   MIT License
 */

#pragma once

#include "demands.hpp"

namespace hf {

    class Actuator {

        friend class Hackflight;
        friend class PidTask;

        // XXX protected:
        public:

            virtual void cut(void) = 0;

            virtual void run(float * demands) = 0;

            virtual void runDisarmed(void) {}

            virtual void setMotorDisarmed(uint8_t index, float value) { (void)index; (void)value; }

            virtual void begin(void) { }

    }; // class Actuator

} // namespace hf
