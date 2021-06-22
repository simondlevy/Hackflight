/*
   Mixer class

   Copyright (c) 2018 Simon D. Levy

   MIT License
 */

#pragma once

#include <RFT_actuator.hpp>
#include <RFT_filters.hpp>
#include <rft_motors/rotary.hpp>

namespace hf {

    class Mixer : public rft::Actuator {

        friend class Hackflight;
        friend class SerialTask;

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

            rft::RotaryMotor * _motors[MAXMOTORS] = {};

            float _previousValues[MAXMOTORS] = {};

            float  _disarmedValues[MAXMOTORS];

            uint8_t _nmotors;

            void writeMotor(uint8_t index, float value)
            {
                // _motors->write(index, value);
            }

            void safeWriteMotor(uint8_t index, float value)
            {
                // Avoid sending the motor the same value over and over
                if (_previousValues[index] != value) {
                    writeMotor(index, value);
                }

                _previousValues[index] = value;
            }

        public:

            typedef enum {

                BRUSHED,
                BRUSHLESS

            } motor_type_t;

        protected:

            motorMixer_t motorDirections[MAXMOTORS];

            Mixer(motor_type_t motortype)
            {
            }

            // For simulation / testing
            Mixer(void)
            {
            }

            void addMotor(rft::Motor * motor, uint8_t pin)
            {
                _motors[_nmotors++] = NULL; // XXX
            }

            // Actuator overrides ----------------------------------------------

            void begin(void) override
            {
                // set disarmed, previous motor values
                for (uint8_t i = 0; i < _nmotors; i++) {
                    _disarmedValues[i] = 0;
                    _previousValues[i] = 0;
                }

                // _motors->begin();
            }

            // This is how we can spin the motors from the GCS
            void runDisarmed(void) override
            {
                for (uint8_t i = 0; i < _nmotors; i++) {
                    safeWriteMotor(i, _disarmedValues[i]);
                }
            }

            void run(float * demands) override
            {
                // Map throttle demand from [-1,+1] to [0,1]
                demands[DEMANDS_THROTTLE] = (demands[DEMANDS_THROTTLE] + 1) / 2;

                float motorvals[MAXMOTORS];

                for (uint8_t i = 0; i < _nmotors; i++) {

                    motorvals[i] = 
                        (demands[DEMANDS_THROTTLE] * motorDirections[i].throttle + 
                         demands[DEMANDS_ROLL]     * motorDirections[i].roll +     
                         demands[DEMANDS_PITCH]    * motorDirections[i].pitch +   
                         demands[DEMANDS_YAW]      * motorDirections[i].yaw);      
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
                    motorvals[i] = rft::Filter::constrainMinMax(motorvals[i], 0, 1);
                }

                for (uint8_t i = 0; i < _nmotors; i++) {
                    safeWriteMotor(i, motorvals[i]);
                }
            }

            void cut(void) override
            {
                for (uint8_t i = 0; i < _nmotors; i++) {
                    writeMotor(i, 0);
                }
            }

            virtual void setMotorDisarmed(uint8_t index, float value) override
            {
                _disarmedValues[index] = value;
            }

    }; // class Mixer

} // namespace hf
