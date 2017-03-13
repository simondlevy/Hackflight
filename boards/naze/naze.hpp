/*
   naze.hpp : Naze32 implementation of routines in board.hpp

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

#include <board.hpp>
#include <hackflight.hpp>

MPU6050 * imu;

SpektrumDSM2048 rx;

BrushlessMotor motors[4];

namespace hf {

class Naze : public Board {

    virtual void dump(char * msg) override
    {
        for (char * c = msg; *c; c++)
            Serial.write(*c);
    }

    virtual void imuRead(int16_t accADC[3], int16_t gyroADC[3]) override
    {
        imu->getMotion6Counts(&accADC[0], &accADC[1], &accADC[2], &gyroADC[0], &gyroADC[1], &gyroADC[2]);
    }


    virtual void init(void) override
    {
        // Init LEDs
        pinMode(3, OUTPUT);
        pinMode(4, OUTPUT);

        Serial.begin(115200);

        Wire.begin(2);

        delay(100);

        motors[0].attach(8);
        motors[1].attach(11);
        motors[2].attach(6);
        motors[3].attach(7);

        imu = new MPU6050();
        imu->begin(AFS_8G, GFS_2000DPS);
    }

    virtual const Config& getConfig() override
    {
        // PIDs
        config.pid.levelP         = 40;
        config.pid.levelI         = 2;
        config.pid.ratePitchrollP = 20;
        config.pid.ratePitchrollI = 15;
        config.pid.ratePitchrollD = 11;
        config.pid.yawP           = 40;
        config.pid.yawI           = 20;

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
        return (int64_t)micros();
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

    virtual void showArmedStatus(bool armed) override
    {
        // XXX this would be a good place to sound a buzzer!
        (void)armed;
    }

    virtual void showAuxStatus(uint8_t status) override
    {
        (void)status;
    }

    virtual void extrasInit(class MSP * _msp) override 
    {
        (void)_msp;
    }

    virtual void extrasCheckSwitch(void) override
    {
    }

    virtual uint8_t extrasGetTaskCount(void) override
    {
        return 0;
    }

    virtual bool extrasHandleMSP(uint8_t command) override
    {
        (void)command;
        return true;
    }

    virtual void extrasPerformTask(uint8_t taskIndex) override
    {
        (void)taskIndex;
    } 

}; // class

} // namespace
