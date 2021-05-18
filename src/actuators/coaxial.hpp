/*
   Mixer subclass for coaxial copters

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include <RFT_actuator.hpp>

#include <Servo.h>

namespace hf {

    class CoaxialActuator : public rft::Actuator {

        private:

            static const uint8_t MAXMOTORS = 20; // arbitrary
            float _motorsPrev[MAXMOTORS] = {};
            float  _motorsDisarmed[MAXMOTORS];

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
            static const uint8_t MOTOR1_PIN = 8;
            static const uint8_t MOTOR2_PIN = 9;

            Servo servo1;
            Servo servo2;

            void initServo(Servo & servo, uint8_t pin)
            {
                servo.attach(pin);
                writeServo(servo, 0);
            }

            void safeWriteMotor(uint8_t index, float value)
            {
                // Avoid sending the motor the same value over and over
                if (_motorsPrev[index] != value) {
                    //writeMotor(index, value);
                }

                _motorsPrev[index] = value;
            }

            void writeServo(Servo & servo, float value)
            {
                // Convert [-.5,+.5] to [0,180]
                servo.write(90 + (int8_t)(100*value));
            }

            void writeServoDisarmed(Servo & servo, uint8_t index)
            {
                writeServo(servo, _motorsDisarmed[index]);
            }

            void initMotor(uint8_t pin)
            {
                digitalWrite(pin, LOW);
            }

            void writeMotorDisarmed(uint8_t pin, uint8_t index)
            {
                analogWrite(pin, (uint8_t)(255*_motorsDisarmed[index]));
            }

        protected:

            virtual void begin(void) override
            {
                initServo(servo1, SERVO1_PIN);
                initServo(servo2, SERVO2_PIN);
            }

            virtual void setMotorDisarmed(uint8_t index, float value) override
            {
                _motorsDisarmed[index] = value;
            }

            virtual void runDisarmed(void) override
            {
                writeServoDisarmed(servo1, 0);
                writeServoDisarmed(servo2, 1);

                writeMotorDisarmed(MOTOR1_PIN, 2);
                writeMotorDisarmed(MOTOR2_PIN, 3);
            }

            virtual void cut(void) override
            {
                // XXX
            }

        public:

            CoaxialActuator(void)
            {
                for (uint8_t i = 0; i < 4; i++) {
                    _motorsDisarmed[i] = 0;
                    _motorsPrev[i] = 0;
                }
            }

            virtual void run(float * demands) override
            {
                Serial.printf("T: %+3.3f    R: %+3.3f    P: %+3.3f    Y: %+3.3f\n",
                        demands[0], demands[1], demands[2], demands[3]);
            }

            virtual uint8_t getType(void) override
            {
                return 1;
            }

    }; // class CoaxialActuator

} // namespace
