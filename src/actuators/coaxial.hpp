/*
   Actuator for coaxial copters

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include <RFT_actuator.hpp>

namespace hf {

    class CoaxialActuator : public rft::Actuator {

        public:

            /*
            MixerCoaxial(rft::Motor * motors) 
                : Mixer(motors, 4)
            {
                //                     Th   RR   PF  YR
                motorDirections[0] = { +1,  0,   0, -1 };   // rotor 1
                motorDirections[1] = { +1,  0,   0, +1 };   // rotor 2
                motorDirections[2] = {  0, +1,   0,  0 };   // servo 1
                motorDirections[3] = {  0,  0 , +1,  0 };   // servo 2
             }
             */

            void run(float * demands)
            {
            }
     };

} // namespace
