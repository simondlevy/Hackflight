/*
   Mixer class

   Copyright (c) 2018 Simon D. Levy

   MIT License
 */

#pragma once

#include "HF_demands.hpp"
#include "HF_motors.hpp"
#include "HF_utils.hpp"

namespace hf {

    class Mixer {

        friend class Hackflight;

        private:

            // Arbitrary
            static const uint8_t MAXMOTORS = 20;

            uint8_t _nmotors = 0;

            static float motorfun(demands_t & demands, demands_t & mix)
            {
                return demands.throttle * mix.throttle + 
                       demands.roll     * mix.roll +     
                       demands.pitch    * mix.pitch +   
                       demands.yaw      * mix.yaw;      
            }

        protected:

            Mixer(uint8_t nmotors)
            {
                _nmotors = nmotors;
            }

        public:

            demands_t spins[MAXMOTORS];

            virtual float constrainMotorValue(uint8_t index, float value)
            {
                (void)index; // all motors behave the same by default
                return constrainMinMax(value, 0, 1);
            }

            void run(demands_t & demands, motors_t & motors)
            {
                // Map throttle demand from [-1,+1] to [0,1]
                demands.throttle = (demands.throttle + 1) / 2;

                for (uint8_t i = 0; i < _nmotors; i++) {

                    motors.values[i] = 
                        (demands.throttle * spins[i].throttle + 
                         demands.roll     * spins[i].roll +     
                         demands.pitch    * spins[i].pitch +   
                         demands.yaw      * spins[i].yaw);      
                }

                float maxMotor = 0;

                for (uint8_t i = 0; i < _nmotors; i++) {
                    maxMotor = motors.values[i] > maxMotor ?
                                 motors.values[i] :
                                 maxMotor;
                }

                for (uint8_t i = 0; i < _nmotors; i++) {

                    // This is a way to still have good gyro corrections if at
                    // least one motor reaches its max
                    motors.values[i] = maxMotor > 1 ?
                                       motors.values[i] - maxMotor + 1 :
                                       motors.values[i];

                    // Keep motor values in appropriate interval
                    motors.values[i] = constrainMotorValue(i, motors.values[i]);
                }
            }

            virtual uint8_t getType(void) 
            {
                return 0;
            }

    }; // class Mixer

} // namespace hf
