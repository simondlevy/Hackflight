/*
   alienflightf3.hpp : AlienflightF3 implementation of routines in board.hpp

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

#include <Arduino.h>
#include <Motor.h>
#include <SpektrumDSM.h>
#include <MPU6050.h>

#include <math.h>

#include <mw32.hpp>
#include <hackflight.hpp>

MPU6050 * mpu;

SpektrumDSM2048 rx;

BrushedMotor motors[4];

namespace hf {

class AlienflightF3 : public MW32 {

    virtual void dump(char * msg) override
    {
        for (char * c = msg; *c; c++)
            Serial.write(*c);
    }

    virtual void imuReadRaw(int16_t accADC[3], int16_t gyroADC[3]) override
    {
        int16_t ax, ay, az, gx, gy, gz;

        if (mpu->getMotion6Counts(&ax, &ay, &az, &gx, &gy, &gz)) {

            accADC[0]  = -ay;
            accADC[1]  = ax;
            accADC[2]  = az;
            gyroADC[0] = -gy;
            gyroADC[1] = gx;
            gyroADC[2] = gz;
        }
    }


    virtual void init(void) override
    {
        // Init LEDs
        pinMode(3, OUTPUT);
        pinMode(4, OUTPUT);

        Serial.begin(115200);

        Wire.begin(2);

        delay(100);

        mpu = new MPU6050();
        mpu->begin(AFS_8G, GFS_2000DPS);

        motors[0].attach(15);
        motors[1].attach(14);
        motors[2].attach(8);
        motors[3].attach(0);

    }

    virtual const Config& getConfig() override
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

    virtual void checkReboot(bool pendReboot) override
    {
        if (pendReboot)
            reset(); // noreturn
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

        digitalWrite(id ? 4 : 3, is_on ? HIGH : LOW);
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

    virtual void reboot(void) override
    {
        resetToBootloader();
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
        motors[index].setSpeed(value);
    }

}; // class

} // namespace
