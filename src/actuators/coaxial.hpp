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


            uint8_t _servo1_pin = 0;
            uint8_t _servo2_pin = 0;
            uint8_t _motor1_pin = 0;
            uint8_t _motor2_pin = 0;

            Servo _servo1;
            Servo _servo2;

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

            void writeMotorDisarmed(uint8_t pin, uint8_t index)
            {
                analogWrite(pin, (uint8_t)(255*_motorsDisarmed[index]));
            }

            void cutMotor(uint8_t pin)
            {
                analogWrite(pin, 0);
            }

        protected:

            virtual void begin(void) override
            {
                initServo(_servo1, _servo1_pin);
                initServo(_servo2, _servo2_pin);
            }

            virtual void setMotorDisarmed(uint8_t index, float value) override
            {
                _motorsDisarmed[index] = value;
            }

            virtual void runDisarmed(void) override
            {
                writeServoDisarmed(_servo1, 0);
                writeServoDisarmed(_servo2, 1);

                writeMotorDisarmed(_motor1_pin, 2);
                writeMotorDisarmed(_motor2_pin, 3);
            }

            virtual void cut(void) override
            {
                cutMotor(_motor1_pin);
                cutMotor(_motor2_pin);
            }

        public:

            CoaxialActuator(uint8_t servo1_pin, uint8_t servo2_pin, uint8_t motor1_pin, uint8_t motor2_pin)
            {
                _servo1_pin = servo1_pin;
                _servo2_pin = servo2_pin;
                _motor1_pin = motor1_pin;
                _motor2_pin = motor2_pin;

                for (uint8_t i = 0; i < 4; i++) {
                    _motorsDisarmed[i] = 0;
                    _motorsPrev[i] = 0;
                }
            }

            virtual void run(float * demands) override
            {
                // XXX
                Serial.printf("T: %+3.3f    R: %+3.3f    P: %+3.3f    Y: %+3.3f\n",
                        demands[0], demands[1], demands[2], demands[3]);
            }

            virtual uint8_t getType(void) override
            {
                return 1;
            }

    }; // class CoaxialActuator

} // namespace
