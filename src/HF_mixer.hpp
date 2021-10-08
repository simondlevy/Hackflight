/*
   Mixer class

   Copyright (c) 2018 Simon D. Levy

   MIT License
 */

#pragma once

#include "HF_demands.hpp"
#include "HF_filters.hpp"

namespace hf {

    class Mixer {

        friend class Hackflight;

        private:

            // Custom mixer data per motor
            typedef struct motorMixer_t {
                int8_t throttle; // T
                int8_t roll; 	 // A
                int8_t pitch;	 // E
                int8_t yaw;	     // R
            } motorMixer_t;

            // Arbitrary
            static const uint8_t MAXMOTORS = 20;

            uint8_t _nmotors = 0;

        protected:

            Mixer(uint8_t nmotors)
            {
                _nmotors = nmotors;
            }

        public:

            motorMixer_t motorDirections[MAXMOTORS];

            virtual float constrainMotorValue(uint8_t index, float value)
            {
                (void)index; // all motors behave the same by default
                return Filter::constrainMinMax(value, 0, 1);
            }

            void run(float * demands, float * motorvals)
            {
                // Map throttle demand from [-1,+1] to [0,1]
                demands[DEMANDS_THROTTLE] = (demands[DEMANDS_THROTTLE] + 1) / 2;

                for (uint8_t i = 0; i < _nmotors; i++) {

                    motorvals[i] = 
                        (demands[DEMANDS_THROTTLE] *
                         motorDirections[i].throttle + 
                         demands[DEMANDS_ROLL] * 
                         motorDirections[i].roll +     
                         demands[DEMANDS_PITCH]
                         * motorDirections[i].pitch +   
                         demands[DEMANDS_YAW]
                         * motorDirections[i].yaw);      
                }

                float maxMotor = motorvals[0];

                for (uint8_t i = 1; i < _nmotors; i++)
                    if (motorvals[i] > maxMotor)
                        maxMotor = motorvals[i];

                for (uint8_t i = 0; i < _nmotors; i++) {

                    // This is a way to still have good gyro corrections if at
                    // least one motor reaches its max
                    if (maxMotor > 1) {
                        motorvals[i] -= maxMotor - 1;
                    }

                    // Keep motor values in appropriate interval
                    motorvals[i] = constrainMotorValue(i, motorvals[i]);
                }
            }

            virtual uint8_t getType(void) 
            {
                return 0;
            }

    }; // class Mixer

} // namespace hf
