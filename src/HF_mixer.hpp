/*
   Mixer class

   Copyright (c) 2018 Simon D. Levy

   MIT License
 */

#pragma once

#include "HF_demands.hpp"

#include "HF_filters.hpp"
#include "HF_motor.hpp"

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

            // XXX make a class for this, or migrate it to rft::Motor
            Motor * _motors[MAXMOTORS] = {};
            float  _disarmedValues[MAXMOTORS];

            uint8_t _nmotors = 0;

            void writeMotor(uint8_t index, float value)
            {
                _motors[index]->write(value);
            }

        public:

            motorMixer_t motorDirections[MAXMOTORS];

            Mixer(void)
            {
                _nmotors = 0;
            }

            void addMotor(Motor * motor)
            {
                _motors[_nmotors++] = motor;
            }

            virtual float constrainMotorValue(uint8_t index, float value)
            {
                (void)index; // all motors behave the same by default
                return Filter::constrainMinMax(value, 0, 1);
            }

            void begin(void)
            {
                // set disarmed motor values
                for (uint8_t i = 0; i < _nmotors; i++) {
                    _disarmedValues[i] = 0;
                    _motors[i]->begin();
                }
            }

            // This is how we can spin the motors from the GCS
            void runDisarmed(void)
            {
                for (uint8_t i = 0; i < _nmotors; i++) {
                    writeMotor(i, _disarmedValues[i]);
                }
            }

            void run(float * demands, bool safe)
            {
                // Don't run motors if its not safe: vehicle should be
                // armed, with throttle above minimum
                if (!safe) {
                    return;
                }

                // Map throttle demand from [-1,+1] to [0,1]
                demands[DEMANDS_THROTTLE] = (demands[DEMANDS_THROTTLE] + 1) / 2;

                printf("%f\n", demands[DEMANDS_PITCH]);

                float motorvals[MAXMOTORS];

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

                for (uint8_t i = 0; i < _nmotors; i++) {
                    writeMotor(i, motorvals[i]);
                }
            }

            void cut(void)
            {
                for (uint8_t i = 0; i < _nmotors; i++) {
                    writeMotor(i, 0);
                }
            }

            virtual void setMotorDisarmed(uint8_t index, float value)
            {
                _disarmedValues[index] = value;
            }

            virtual uint8_t getType(void) 
            {
                return 0;
            }

    }; // class Mixer

} // namespace hf
