/*
   Mixer subclass for coaxial copters

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include <RFT_actuator.hpp>

#include <Servo.h>

namespace hf {

    class MixerCoaxial : public rft::Actuator {

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

            virtual void runDisarmed(void) override
            {
                // XXX
            }

            virtual void cut(void) override
            {
                // XXX
            }

        public:

            virtual void run(float * demands) override
            {
                Serial.printf("T: %+3.3f    R: %+3.3f    P: %+3.3f    Y: %+3.3f\n",
                        demands[0], demands[1], demands[2], demands[3]);
            }

            virtual uint8_t getType(void) override
            {
                return 1;
            }
    };

} // namespace
