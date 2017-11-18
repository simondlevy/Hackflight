/*
   ladybug.hpp : Ladybug Flight Controller implementation of Hackflight Board routines

   Uses EM7180 SENtral Sensor Hub in master mode mode

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
#include <EM7180.h>
#include <stdarg.h>
#include "hackflight.hpp"

namespace hf {

class Ladybug : public Board {

    private:

        uint8_t _motorPins[4] = {13, A2, 3, 11};

        float _eulerAnglesRadians[3];

        EM7180 _sentral;

        static const uint16_t GYRO_RES_DEGREES_PER_SECOND = 2000;

    protected:

        virtual void init(Config &config) override
        {
            // Begin serial comms
            Serial.begin(115200);

            // Setup LEDs and turn them off
            pinMode(A1, OUTPUT);
            digitalWrite(A1, LOW);

            // Start I^2C
            Wire.begin();

            // Hang a bit before starting up the EM7180
            delay(100);

            // Start the EM7180: ranges = accel +/-8G, gyro +/-2000 deg/sec; mag +/-1000 Tesla
            uint8_t status = _sentral.begin(8, 2000, 1000);
            while (status) {
                Serial.println(EM7180::errorToString(status));
            }

            // Initialize the motors
            for (int k=0; k<4; ++k) {
                analogWriteFrequency(_motorPins[k], 10000);  
                analogWrite(_motorPins[k], 0);  
            }
        }

        virtual void delayMilliseconds(uint32_t msec) override
        {
            delay(msec);
        }

        virtual uint64_t getMicros() override
        {
            return (uint64_t)micros();
        }

        virtual void ledSet(uint8_t id, bool is_on) override
        { 
            (void)id;

            digitalWrite(A1, is_on ? HIGH : LOW);
        }

        virtual uint8_t serialAvailableBytes(void) override
        {
            return Serial.available();
        }

        virtual uint8_t serialReadByte(void) override
        {
            return Serial.read();
        }

        virtual void serialWriteByte(uint8_t c) override
        {
            Serial.write(c);
        }

        virtual void writeMotor(uint8_t index, float value) override
        {
            // Scale motor value from [0,1] to [0,255]
            uint8_t aval = (uint8_t)(value * 255);

            // Avoid sending the motor the same value over and over
            static uint8_t avalPrev[4];

            if (aval != avalPrev[index]) {
                analogWrite(_motorPins[index], aval);
            }

            avalPrev[index] = aval;
        }

        virtual void extrasImuPoll(void) override
        {
            uint8_t errorStatus = _sentral.poll();

            if (errorStatus) {
                Serial.print("ERROR: ");
                Serial.println(EM7180::errorToString(errorStatus));
                return;
            }
        }

        virtual void getImu(float eulerAnglesRadians[3], float gyroRadiansPerSecond[3]) override
        {
            static float q[4];
            _sentral.getQuaternions(q);

            float yaw   = atan2(2.0f * (q[0] * q[1] + q[3] * q[2]), q[3] * q[3] + q[0] * q[0] - q[1] * q[1] - q[2] * q[2]);   
            float pitch = -asin(2.0f * (q[0] * q[2] - q[3] * q[1]));
            float roll  = atan2(2.0f * (q[3] * q[0] + q[1] * q[2]), q[3] * q[3] - q[0] * q[0] - q[1] * q[1] + q[2] * q[2]);

            // Also store Euler angles for extrasUpdateAccelZ()
            _eulerAnglesRadians[0] =  eulerAnglesRadians[0] = roll;
            _eulerAnglesRadians[1] =  eulerAnglesRadians[1] = -pitch; // compensate for IMU orientation
            _eulerAnglesRadians[2] =  eulerAnglesRadians[2] = yaw;

            int16_t gyroRaw[3];

            _sentral.getGyroRaw(gyroRaw[0], gyroRaw[1], gyroRaw[2]);

            gyroRaw[1] = -gyroRaw[1];
            gyroRaw[2] = -gyroRaw[2];

            for (uint8_t k=0; k<3; ++k) {
                gyroRadiansPerSecond[k] = (float)GYRO_RES_DEGREES_PER_SECOND * gyroRaw[k] / (1<<15); // raw to degrees
                gyroRadiansPerSecond[k] = M_PI * gyroRadiansPerSecond[k] / 180.; // degrees to radians
            }
        }

        virtual bool extrasHaveBaro(void) override
        { 
            return true; 
        }

        virtual float extrasGetBaroPressure(void) override 
        {
            float pressure, temperature;
            _sentral.getBaro(pressure, temperature);
            return pressure;
        }

        virtual void extrasImuGetAccel(float accelGs[3]) override
        {
            int16_t accelRaw[3];
            _sentral.getAccelRaw(accelRaw[0], accelRaw[1], accelRaw[2]);
            for (uint8_t k=0; k<3; ++k) {
                accelGs[k] = accelRaw[k] / 2048.f;
            }
        }

}; // class

} // namespace
