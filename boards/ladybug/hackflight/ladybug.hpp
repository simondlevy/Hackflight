/*
   ladybug.hpp : STM32L432 implementation of routines in board.hpp

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

#include <SpektrumDSM.h>
#include <EM7180.h>

#include "hackflight.hpp"

SpektrumDSM2048 rx;

EM7180 imu;

static uint8_t motorPins[4] = {11, 1, 3, 13};

namespace hf {

class Ladybug : public Board {

    virtual void dump(char * msg) override
    {
        Serial.print(msg);
    }

    virtual void init(void) override
    {
        // Begin serial comms
        Serial.begin(115200);

        // Setup LED and turn it off
        pinMode(A1, OUTPUT);
        digitalWrite(A1, LOW);

        Wire.begin();        

        delay(1000);

        // Start the EM7180
        uint8_t status = imu.begin(8, 2000, 1000);
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
        config.pid.levelP         = 20;
        config.pid.levelI         = 1;
        config.pid.ratePitchrollP = 18;
        config.pid.ratePitchrollI = 15;
        config.pid.ratePitchrollD = 12;
        config.pid.yawP           = 85;
        config.pid.yawI           = 45;

        // "Software trim"
        config.pid.softwareTrim[AXIS_PITCH] = -50;

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

    virtual bool rcSerialReady(void) override
    {
        return rx.frameComplete();
    }

    virtual bool rcUseSerial(void) override
    {
        rx.begin();
        return true;
    }

    virtual uint16_t rcReadSerial(uint8_t chan) override
    {
        uint8_t chanmap[5] = {1, 2, 3, 0, 5};
        return rx.readRawRC(chanmap[chan]);
    }

    virtual uint16_t rcReadPwm(uint8_t chan) override
    {
        (void)chan;
        return 0;
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
        uint8_t aval = map(value, config.pwm.min, config.pwm.max, 0, 255);

        analogWrite(motorPins[index], aval);
    }

    virtual void imuUpdate(void) override
    {
        uint8_t errorStatus = imu.update();
        
        if (errorStatus) {
            Serial.print("ERROR: ");
            Serial.println(EM7180::errorToString(errorStatus));
            return;
        }
    }
        
    virtual void imuGetEulerAngles(float dT_sec, int16_t accelSmooth[3], int16_t gyroRaw[3], float eulerAnglesRadians[3]) override
    {
        // We ignore these values inputs, using quaternions instead
        (void)dT_sec;
        (void)accelSmooth;
        (void)gyroRaw;
        
        static float q[4];
        imu.getQuaternions(q);

        float yaw   = atan2(2.0f * (q[0] * q[1] + q[3] * q[2]), q[3] * q[3] + q[0] * q[0] - q[1] * q[1] - q[2] * q[2]);   

        float pitch = -asin(2.0f * (q[0] * q[2] - q[3] * q[1]));
        float roll  = atan2(2.0f * (q[3] * q[0] + q[1] * q[2]), q[3] * q[3] - q[0] * q[0] - q[1] * q[1] + q[2] * q[2]);

        eulerAnglesRadians[0] =  roll;
        eulerAnglesRadians[1] = -pitch; // compensate for IMU orientation
        eulerAnglesRadians[2] =  yaw;
    }

    virtual void imuReadRaw(int16_t accelRaw[3], int16_t gyroRaw[3]) override
    {
         imu.getAccelRaw(accelRaw[0], accelRaw[1], accelRaw[2]);
         imu.getGyroRaw(gyroRaw[0], gyroRaw[1], gyroRaw[2]);

         gyroRaw[1] = -gyroRaw[1];
         gyroRaw[2] = -gyroRaw[2];
    }

}; // class

} // namespace
