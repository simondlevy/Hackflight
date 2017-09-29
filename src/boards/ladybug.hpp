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

#include "hackflight.hpp"

namespace hf {

class Ladybug : public Board {

    private:

        uint8_t _motorPins[4] = {13, A2, 3, 11};

        float _eulerAngles[3];

        EM7180 _sentral;

    protected:

        virtual void init(Config &config) override
        {
            // Accelerometer reading at 1G (vehicle resting flat)
            config.altitude.accel.oneG = 2048; // default is 4096

            // Begin serial comms
            Serial.begin(115200);

            // Setup LEDs and turn them off
            pinMode(A1, OUTPUT);
            digitalWrite(A1, LOW);

            // Start I^2C
            Wire.begin();

            // Hang a bit before starting up the EM7180
            delay(100);

            // Start the EM7180
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

        virtual void writeMotor(uint8_t index, uint16_t value) override
        {
            uint8_t aval = map(value, 1000, 2000, 0, 255);

            // Avoid sending the motor the same value over and over
            static uint8_t avalPrev[4];

            if (aval != avalPrev[index]) {
                analogWrite(_motorPins[index], aval);
            }

            //Serial.print(index+1); Serial.print(": "); Serial.print(aval); Serial.print(index==3?"\n":"\t"); 

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

        virtual void getImu(float eulerAngles[3], int16_t gyroRaw[3]) override
        {
            static float q[4];
            _sentral.getQuaternions(q);

            float yaw   = atan2(2.0f * (q[0] * q[1] + q[3] * q[2]), q[3] * q[3] + q[0] * q[0] - q[1] * q[1] - q[2] * q[2]);   
            float pitch = -asin(2.0f * (q[0] * q[2] - q[3] * q[1]));
            float roll  = atan2(2.0f * (q[3] * q[0] + q[1] * q[2]), q[3] * q[3] - q[0] * q[0] - q[1] * q[1] + q[2] * q[2]);

            // Also store Euler angles for extrasUpdateAccelZ()
            _eulerAngles[0] =  eulerAngles[0] = roll;
            _eulerAngles[1] =  eulerAngles[1] = -pitch; // compensate for IMU orientation
            _eulerAngles[2] =  eulerAngles[2] = yaw;

            _sentral.getGyroRaw(gyroRaw[0], gyroRaw[1], gyroRaw[2]);
            gyroRaw[1] = -gyroRaw[1];
            gyroRaw[2] = -gyroRaw[2];
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

        virtual void extrasImuGetAccel(int16_t accelRaw[3]) override
        {
            _sentral.getAccelRaw(accelRaw[0], accelRaw[1], accelRaw[2]);
        }

}; // class

} // namespace
