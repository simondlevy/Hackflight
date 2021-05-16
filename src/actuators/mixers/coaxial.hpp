/*
   Mixer subclass for coaxial copters

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include <RFT_motor.hpp>
#include "actuators/mixer.hpp"

namespace hf {

    class MixerCoaxial : public Mixer {

        private:

            class CoaxialMotor : public rft::Motor {

                public:

                    CoaxialMotor(void)
                        : rft::Motor(4) 
                    {
                    }

                    virtual void write(uint8_t index, float value) override
                    {
                    }

            }; 

            CoaxialMotor _motors;

        public:

            MixerCoaxial(void) 
                : Mixer(&_motors, 4)
            {
                //                     Th  RR  PF  YR
                motorDirections[0] = {  0, -1, -1,  0 };    // Servo 1
                motorDirections[1] = {  0, +1, +1,  0 };    // Servo 2
                motorDirections[2] = { +1,  0,  0, +1 };    // Motor 1
                motorDirections[3] = { +1,  0,  0, -1 };    // Motor 2
            }

            virtual uint8_t getType(void) override
            {
                return 1;
            }
    };

} // namespace
