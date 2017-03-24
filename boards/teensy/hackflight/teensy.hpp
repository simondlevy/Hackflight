/*
   teensy.hpp : Teensy3.2 implementation of routines in board.hpp

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
#include <EM7180_passthru.h>

#include <math.h>

#include "mw32.hpp"
#include "hackflight.hpp"

SpektrumDSM2048 rx;

EM7180_passthru em7180;

static uint8_t motorPins[4] = {9, 22, 5, 23};

namespace hf {

class Teensy : public MW32 {

    virtual void dump(char * msg) override
    {
        Serial.printf("%s", msg);
    }

    virtual void init(void) override
    {
        // Setup LEDs and turn them off
        pinMode(27, OUTPUT);
        pinMode(29, OUTPUT);
        digitalWrite(27, HIGH);
        digitalWrite(29, HIGH);

        // Setup for Master mode, pins 18/19, external pullups, 400kHz for Teensy 3.1
        Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
        delay(1000);

        // Start the EM7180 (for now, only specify IMU params)
        em7180.begin(AFS_8G, GFS_2000DPS);

        // Begin serial comms
        Serial.begin(115200);

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

    virtual void checkReboot(bool pendReboot) override
    {
        (void)pendReboot;
    }

    virtual void delayMilliseconds(uint32_t msec) override
    {
        delay(msec);
    }

    virtual uint64_t getMicros() override
    {
        return (uint64_t)micros();
    }

    virtual void imuReadRaw(int16_t accelADC[3], int16_t gyroADC[3]) override
    {
        int16_t a[3];
        int16_t g[3];

        em7180.readAccelData(a); 
        em7180.readGyroData(g);  

        accelADC[0] =  a[1];
        accelADC[1] = -a[0];
        accelADC[2] =  a[2];

        gyroADC[0] =  g[1];
        gyroADC[1] = -g[0];
        gyroADC[2] =  g[2];

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

    virtual void reboot(void) override
    {
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

}; // class

} // namespace
