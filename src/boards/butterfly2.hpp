/*
   butterfly2.hpp : Implementation of Hackflight Board routines for Butterfly
                   dev board + EM7180 SENtral Sensor Fusion Solution + 
                   brushless motors

   Additional libraries required: https://github.com/simondlevy/EM7180
                                  https://github.com/simondlevy/CrossPlatformI2C

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
#include <Servo.h>

#include <EM7180.h> 

#include "filters.hpp"
#include "hackflight.hpp"
#include "realboard.hpp"

namespace hf {

    class Butterfly : public RealBoard {

        private:

            // Tunable EM7180 parameters
            static const uint8_t  ARES           = 8;    // Gs
            static const uint16_t GRES           = 2000; // degrees per second
            static const uint16_t MRES           = 1000; // microTeslas
            static const uint8_t  MAG_RATE       = 100;  // Hz
            static const uint16_t ACCEL_RATE     = 330;  // Hz
            static const uint16_t GYRO_RATE      = 330;  // Hz
            static const uint8_t  BARO_RATE      = 50;   // Hz
            static const uint8_t  Q_RATE_DIVISOR = 5;    // 1/5 gyro rate

             // Motor pins
            const uint8_t MOTOR_PINS[4] = {3, 4, 5, 6};

            // Min, max PWM values
            const uint16_t PWM_MIN = 990;
            const uint16_t PWM_MAX = 2000;

            // Butterfly board follows Arduino standard for LED pin
            const uint8_t LED_PIN = 13;

            // Run motor ESCs using standard Servo library
            Servo _escs[4];

            float _gyroAdcToRadians;

            EM7180_Master _sentral = EM7180_Master(ARES, GRES, MRES, MAG_RATE, ACCEL_RATE, GYRO_RATE, BARO_RATE, Q_RATE_DIVISOR);

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

 
        protected:

            void delayMilliseconds(uint32_t msec)
            {
                delay(msec);
            }

            void ledSet(bool is_on)
            { 
                digitalWrite(LED_PIN, is_on ? LOW : HIGH);
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

            void writeMotor(uint8_t index, float value)
            {
                _escs[index].writeMicroseconds((uint16_t)(PWM_MIN+value*(PWM_MAX-PWM_MIN)));
            }

            virtual uint32_t getMicroseconds(void) override
            {
                return micros();
            }

            void delaySeconds(float sec)
            {
                delay((uint32_t)(1000*sec));
            }

            bool getGyrometer(float gyroRates[3])
            {
                // Since gyro is updated most frequently, use it to drive SENtral polling
                checkEventStatus();

                if (_sentral.gotGyrometer()) {

                    int16_t gx, gy, gz;

                    _sentral.readGyrometer(gx, gy, gz);

                    // invert pitch, yaw gyro direction to keep other code simpler
                    gy = -gy;
                    gz = -gz;

                    gyroRates[0] = gx * _gyroAdcToRadians;
                    gyroRates[1] = gy * _gyroAdcToRadians;
                    gyroRates[2] = gz * _gyroAdcToRadians;

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

            bool getAccelerometer(float accelGs[3])
            {
                if (_sentral.gotAccelerometer()) {
                    int16_t ax, ay, az;
                    _sentral.readAccelerometer(ax, ay, az);
                    accelGs[0] = ax / 2048.f;
                    accelGs[1] = ay / 2048.f;
                    accelGs[2] = az / 2048.f;
                    return true;
                }

                return false;
            }

            bool getBarometer(float & pressure)
            {
                if (_sentral.gotBarometer()) {
                    float temperature; // ignored
                    _sentral.readBarometer(pressure, temperature);
                    return true;
                }

                return false;
            }


        public:

            Butterfly(void)
            {
                // Use D30,31 as GND, 3.3V
                pinMode(30, OUTPUT);
                digitalWrite(30, LOW);
                pinMode(31, OUTPUT);
                digitalWrite(31, HIGH);

                // Begin serial comms
                Serial.begin(115200);

                // Setup LED pin and turn it off
                pinMode(LED_PIN, OUTPUT);
                digitalWrite(LED_PIN, HIGH);

                // Connect to the ESCs and send them the baseline values
                for (uint8_t k=0; k<4; ++k) {
                    _escs[k].attach(MOTOR_PINS[k]);
                    _escs[k].writeMicroseconds(PWM_MIN);
                }

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

                // Get actual gyro rate for conversion to radians
                uint8_t accFs=0; uint16_t gyroFs=0; uint16_t magFs=0;
                _sentral.getFullScaleRanges(accFs, gyroFs, magFs);
                _gyroAdcToRadians = M_PI * (float)gyroFs / (1<<15) / 180.;  

                // Hang a bit more
                delay(100);

                // Do general real-board initialization
                RealBoard::init();
            }


    }; // class Butterfly

    void Board::outbuf(char * buf)
    {
        Serial.print(buf);
    }

} // namespace hf
