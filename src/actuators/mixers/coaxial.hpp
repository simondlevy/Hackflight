/*
   Mixer subclass for coaxial copters

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include <RFT_motor.hpp>
#include "actuators/mixer.hpp"

#include <Servo.h>

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

            // XXX hard-code for now
            static const uint8_t SERVO1_PIN = 22;
            static const uint8_t SERVO2_PIN = 23;

            Servo servo1;
            Servo servo2;

            void initServo(Servo & servo, uint8_t pin)
            {
                servo.attach(pin);
                servo.write(90);
            }

        protected:

            virtual void begin(void) override
            {
                initServo(servo1, SERVO1_PIN);
                initServo(servo2, SERVO2_PIN);
            }

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
