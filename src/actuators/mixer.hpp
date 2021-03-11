/*
   Mixer class

   Copyright (c) 2018 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MEReceiverHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "filters.hpp"
#include "motor.hpp"
#include "actuator.hpp"
#include "demands.hpp"

namespace hf {

    class Mixer : public Actuator {

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

            float _motorsPrev[MAXMOTORS] = {};

            float  _motorsDisarmed[MAXMOTORS] = {};

            void writeMotor(uint8_t index, float value)
            {
                _motors->write(index, value);
            }

            void safeWriteMotor(uint8_t index, float value)
            {
                // Avoid sending the motor the same value over and over
                if (_motorsPrev[index] != value) {
                    writeMotor(index, value);
                }

                _motorsPrev[index] = value;
            }

        protected:

            Motor * _motors;

            motorMixer_t motorDirections[MAXMOTORS];

            Mixer(Motor * motors, uint8_t nmotors)
            {
                _motors = motors;
                _nmotors = nmotors;

                // set disarmed, previous motor values
                for (uint8_t i = 0; i < nmotors; i++) {
                    _motorsDisarmed[i] = 0;
                    _motorsPrev[i] = 0;
                }

            }

            uint8_t _nmotors;

            virtual void begin(void) override
            {
                _motors->begin();
            }

            // This is how we can spin the motors from the GCS
            virtual void runDisarmed(void) override
            {
                for (uint8_t i = 0; i < _nmotors; i++) {
                    safeWriteMotor(i, _motorsDisarmed[i]);
                }
            }

            virtual void setMotorDisarmed(uint8_t index, float value) override
            {
                _motorsDisarmed[index] = value;
            }

            // This helps support servos
            virtual float constrainMotorValue(uint8_t index, float value) 
            {
                (void)index;
                return Filter::constrainMinMax(value, 0, 1);
            }

            virtual void cut(void) override
            {
                for (uint8_t i = 0; i < _nmotors; i++) {
                    writeMotor(i, 0);
                }
            }

        public:

            virtual void run(demands_t demands) override
            {
                // Map throttle demand from [-1,+1] to [0,1]
                demands.throttle = (demands.throttle + 1) / 2;

                float motorvals[MAXMOTORS];

                for (uint8_t i = 0; i < _nmotors; i++) {

                    motorvals[i] = 
                        (demands.throttle * motorDirections[i].throttle + 
                         demands.roll     * motorDirections[i].roll +     
                         demands.pitch    * motorDirections[i].pitch +   
                         demands.yaw      * motorDirections[i].yaw);      
                }

                float maxMotor = motorvals[0];

                for (uint8_t i = 1; i < _nmotors; i++)
                    if (motorvals[i] > maxMotor)
                        maxMotor = motorvals[i];

                for (uint8_t i = 0; i < _nmotors; i++) {

                    // This is a way to still have good gyro corrections if at least one motor reaches its max
                    if (maxMotor > 1) {
                        motorvals[i] -= maxMotor - 1;
                    }

                    // Keep motor values in appropriate interval
                    motorvals[i] = constrainMotorValue(i, motorvals[i]);
                }

                for (uint8_t i = 0; i < _nmotors; i++) {
                    safeWriteMotor(i, motorvals[i]);
                }
            }

    }; // class Mixer

} // namespace hf
