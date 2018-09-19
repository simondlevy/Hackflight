/*
   ladybug.hpp : Ladybug Flight Controller implementation of Hackflight Board routines

   Uses EM7180 SENtral Sensor Hub in master mode mode

   Copyright (c) 2018 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <Wire.h>
#include <EM7180_Master.h>
#include <stdarg.h>
#include "hackflight.hpp"
#include "realboard.hpp"


namespace hf {

    class Ladybug : public RealBoard {

        private:

            // Tunable EM7180 parameters
            static const uint8_t  MAG_RATE       = 100;  // Hz
            static const uint16_t ACCEL_RATE     = 330;  // Hz
            static const uint16_t GYRO_RATE      = 330;  // Hz
            static const uint8_t  BARO_RATE      = 50;   // Hz
            static const uint8_t  Q_RATE_DIVISOR = 5;    // 1/5 gyro rate

            // Tindie version has LED on pin A4, but we support older version with LED on pin A1
            uint8_t _led_pin;

            const uint8_t MOTOR_PINS[4] = {13, A2, 3, 11};

            EM7180_Master _sentral = EM7180_Master(MAG_RATE, ACCEL_RATE, GYRO_RATE, BARO_RATE, Q_RATE_DIVISOR);

            void checkEventStatus(void)
            {
                _sentral.checkEventStatus();

                if (_sentral.gotError()) {
                    while (true) {
                        Serial.print("ERROR: ");
                        Serial.println(_sentral.getErrorString());
                    }
                }
            }

            void delaySeconds(float sec)
            {
                delay((uint32_t)(1000*sec));
            }

            void setLed(bool isOn)
            { 
                digitalWrite(_led_pin, isOn ? HIGH : LOW);
            }

            uint8_t serialAvailableBytes(void)
            {
                return Serial.available();
            }

            uint8_t serialReadByte(void)
            {
                return Serial.read();
            }

            void serialWriteByte(uint8_t c)
            {
                Serial.write(c);
            }

            virtual uint32_t getMicroseconds(void) override
            {
                return micros();
            }

            void writeMotor(uint8_t index, float value)
            {
                // Scale motor value from [0,1] to [0,255]
                analogWrite(MOTOR_PINS[index], (uint8_t)(value * 255));
            }

            bool getGyrometer(float gyro[3])
            {
                // Since gyro is updated most frequently, use it to drive SENtral polling
                checkEventStatus();

                if (_sentral.gotGyrometer()) {

                    float gx, gy, gz;

                    // Returns degrees / sec
                    _sentral.readGyrometer(gx, gy, gz);

                    // Convert degrees / sec to radians / sec
                    gyro[0] = radians(gx);
                    gyro[1] = radians(gy);
                    gyro[2] = radians(gz);

                    return true;
                }

                return false;
            }

            bool getQuaternion(float quat[4])
            {
                if (_sentral.gotQuaternion()) {

                    static float qw, qx, qy, qz;

                    _sentral.readQuaternion(qw, qx, qy, qz);

                    quat[0] = qw;
                    quat[1] = qx;
                    quat[2] = qy;
                    quat[3] = qz;

                    return true;
                }

                return false;
            }

        public:

            // Support prototype version where LED is on pin A1
            Ladybug(uint8_t ledPin = A4) 
            {
                _led_pin = ledPin;

                // Begin serial comms
                Serial.begin(115200);

                // Setup LEDs and turn them off
                pinMode(_led_pin, OUTPUT);
                digitalWrite(_led_pin, LOW);

                // Start I^2C
                Wire.begin();

                // Hang a bit before starting up the EM7180
                delay(100);

                // Start the EM7180 in master mode, no interrupt
                if (!_sentral.begin()) {
                    while (true) {
                        Serial.println(_sentral.getErrorString());
                    }
                }

                // Initialize the motors
                for (int k=0; k<4; ++k) {
                    analogWriteFrequency(MOTOR_PINS[k], 10000);  
                    analogWrite(MOTOR_PINS[k], 0);  
                }

                // Hang a bit more
                delay(100);

                // Do general real-board initialization
                RealBoard::init();
            }

    }; // class Ladybug

    void Board::outbuf(char * buf)
    {
        Serial.print(buf);
    }

} // namespace hf
