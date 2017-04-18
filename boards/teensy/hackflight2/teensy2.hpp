/*
   teensy.hpp : Teensy3.2 implementation of routines in board.hpp

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

#include <i2c_t3.h>
#include <SpektrumDSM.h>
#include <EM7180.h>

#include "hackflight.hpp"

SpektrumDSM2048 rx;

EM7180 em7180;

static uint8_t motorPins[4] = {9, 22, 5, 23};

namespace hf {

class Teensy2 : public Board {

    virtual void dump(char * msg) override
    {
        Serial.printf("%s", msg);
    }

    virtual void init(void) override
    {
        // Begin serial comms
        Serial.begin(115200);

        // Setup LEDs and turn them off
        pinMode(27, OUTPUT);
        pinMode(29, OUTPUT);
        digitalWrite(27, HIGH);
        digitalWrite(29, HIGH);

        // Setup for Master mode, pins 18/19, external pullups, 400kHz for Teensy 3.1
        Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
        delay(1000);

        // Start the EM7180
        uint8_t status = em7180.begin(8, 2000, 1000);
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
        config.pid.levelP         = 40;
        config.pid.levelI         = 2;
        config.pid.ratePitchrollP = 36;
        config.pid.ratePitchrollI = 30;
        config.pid.ratePitchrollD = 23;
        config.pid.yawP           = 85;
        config.pid.yawI           = 45;

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
        (void)max_brightness;

        digitalWrite(id ? 29 : 27, is_on ? LOW : HIGH); // NB: on = LOW; off = HIGH
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

    virtual void imuInit(void) override
    {
    }

    virtual void imuRestartCalibration(void) override
    {
    }

    virtual bool imuAccelCalibrated(void) override
    {
        return true;
    }

    virtual bool imuGyroCalibrated(void) override
    {
        return true;
    }

    virtual void imuUpdate(uint32_t currentTime, bool armed) override
    {
        (void)currentTime;
        (void)armed;

        uint8_t errorStatus = em7180.update();
        
        if (errorStatus) {
            Serial.printf("ERROR: %s\n", EM7180::errorToString(errorStatus));
            return;
        }
    }
        

    virtual void imuGetEulerAngles(int16_t eulerAngles[3]) override
    {

        static float q[4];
        em7180.getQuaternions(q);

        float yaw   = atan2(2.0f * (q[0] * q[1] + q[3] * q[2]), q[3] * q[3] + q[0] * q[0] - q[1] * q[1] - q[2] * q[2]);   

        float pitch = -asin(2.0f * (q[0] * q[2] - q[3] * q[1]));
        float roll  = atan2(2.0f * (q[3] * q[0] + q[1] * q[2]), q[3] * q[3] - q[0] * q[0] - q[1] * q[1] + q[2] * q[2]);
        
        pitch *= 180.0f / PI;
        
        yaw   *= 180.0f / PI; 
        if(yaw < 0) yaw   += 360.0f ; // Ensure yaw stays between 0 and 360
        
        roll  *= 180.0f / PI;

        eulerAngles[0] = int16_t(roll * 10);
        eulerAngles[1] = int16_t(pitch * 10);
        eulerAngles[2] = int16_t(yaw);
    }

    virtual void imuGetRawGyro(int16_t gyroRaw[3]) override
    {
         em7180.getGyroRaw(gyroRaw[0], gyroRaw[1], gyroRaw[2]);
    }



}; // class

} // namespace
