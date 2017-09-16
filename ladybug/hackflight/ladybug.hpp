/*
   ladybug.hpp : Ladybug Flight Controller implementation of routines in board.hpp

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

        uint8_t motorPins[4] = {13, A2, 3, 11};

        float eulerAngles[3];

        EM7180 sentral;

    protected:

        virtual void debug(char * msg) override
        {
            Serial.print(msg);
        }

        virtual void init(void) override
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

            // Start the EM7180
            uint8_t status = sentral.begin(8, 2000, 1000);
            while (status) {
                Serial.println(EM7180::errorToString(status));
            }

            // Initialize the motors
            for (int k=0; k<4; ++k) {
                analogWriteFrequency(motorPins[k], 10000);  
                analogWrite(motorPins[k], 0);  
            }
        }

        virtual const Config& getConfig(void) override
        {
            // PIDs
            config.stabilize.levelP         = 0.20f;

            config.stabilize.ratePitchrollP = 0.225f;
            config.stabilize.ratePitchrollI = 0.001875f;
            config.stabilize.ratePitchrollD = 0.375f;

            config.stabilize.yawP           = 1.0625f;
            config.stabilize.yawI           = 0.005625f;

            // "Software trim"
            //config.stabilize.softwareTrim[AXIS_ROLL]  = +37;

            // Altitude-hold
            config.altitude.accel.oneG = 2048;

            return config;
        }

        virtual void delayMilliseconds(uint32_t msec) override
        {
            delay(msec);
        }

        virtual uint64_t getMicros() override
        {
            return (uint64_t)micros();
        }

        virtual void ledSet(uint8_t id, bool is_on, float max_brightness) override
        { 
            (void)id;
            (void)max_brightness;

            digitalWrite(A1, is_on ? HIGH : LOW);
        }

        // Implemented in cppm.hpp, dsmx.hpp
        virtual void rcInit(void) override;
        virtual bool rcUseSerial(void) override;
        virtual uint16_t rcReadChannel(uint8_t chan) override;

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
            uint8_t aval = map(value, config.pwm.min, config.pwm.max, 0, 255);

            // Avoid sending the motor the same value over and over
            static uint8_t avalPrev[4];

            if (aval != avalPrev[index]) {
                analogWrite(motorPins[index], aval);
            }

            /*
            Serial.print(index);
            Serial.print(": ");
            Serial.print(aval);
            Serial.print(index==3?"\n":"\t");
            */
            avalPrev[index] = aval;
        }

        virtual void extrasImuPoll(void) override
        {
            uint8_t errorStatus = sentral.poll();

            if (errorStatus) {
                Serial.print("ERROR: ");
                Serial.println(EM7180::errorToString(errorStatus));
                return;
            }
        }

        virtual void imuGetEuler(float _eulerAngles[3]) override
        {
            static float q[4];
            sentral.getQuaternions(q);

            float yaw   = atan2(2.0f * (q[0] * q[1] + q[3] * q[2]), q[3] * q[3] + q[0] * q[0] - q[1] * q[1] - q[2] * q[2]);   
            float pitch = -asin(2.0f * (q[0] * q[2] - q[3] * q[1]));
            float roll  = atan2(2.0f * (q[3] * q[0] + q[1] * q[2]), q[3] * q[3] - q[0] * q[0] - q[1] * q[1] + q[2] * q[2]);

            _eulerAngles[0] =  roll;
            _eulerAngles[1] = -pitch; // compensate for IMU orientation
            _eulerAngles[2] =  yaw;

            // Store Euler angles for extrasUpdateAccelZ()
            memcpy(eulerAngles, _eulerAngles, 3*sizeof(float));
        }

        virtual void imuGetGyro(int16_t gyroRaw[3]) override
        {
            sentral.getGyroRaw(gyroRaw[0], gyroRaw[1], gyroRaw[2]);
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
            sentral.getBaro(pressure, temperature);
            return pressure;
        }

        virtual void extrasImuGetAccel(int16_t accelRaw[3]) override
        {
            sentral.getAccelRaw(accelRaw[0], accelRaw[1], accelRaw[2]);
        }

}; // class

} // namespace
