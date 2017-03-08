/*
   alienflight.hpp : AlienflightF3 implementation of routines in board.hpp

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

#include <Arduino.h>
#include <Motor.h>
#include <SpektrumDSM.h>
#include <MPU6050.h>

#include <math.h>

#include <board.hpp>
#include <hackflight.hpp>

#define IMU_LOOPTIME_USEC       3500
#define CALIBRATING_GYRO_MSEC   3500

MPU6050 * imu;

SpektrumDSM2048 rx;

BrushedMotor motors[4];

namespace hf {

class AlienflightF3 : public Board {

    void dump(char * msg)
    {
        for (char * c = msg; *c; c++)
            Serial.write(*c);
    }

    void imuRead(int16_t accADC[3], int16_t gyroADC[3])
    {
        int16_t ax, ay, az, gx, gy, gz;

        if (imu->getMotion6Counts(&ax, &ay, &az, &gx, &gy, &gz)) {

            accADC[0]  = -ay;
            accADC[1]  = ax;
            accADC[2]  = az;
            gyroADC[0] = -gy;
            gyroADC[1] = gx;
            gyroADC[2] = gz;
        }
    }


    void init(uint16_t & acc1G, float & gyroScale, uint32_t & looptimeMicroseconds, uint32_t & calibratingGyroMsec)
    {
        // Init LEDs
        pinMode(3, OUTPUT);
        pinMode(4, OUTPUT);

        Serial.begin(115200);

        Wire.begin(2);

        motors[0].attach(15);
        motors[1].attach(14);
        motors[2].attach(8);
        motors[3].attach(0);

        looptimeMicroseconds = IMU_LOOPTIME_USEC;
        calibratingGyroMsec  = CALIBRATING_GYRO_MSEC;

        imu = new MPU6050();

        imu->begin(AFS_8G, GFS_2000DPS);

        // Accel scale 8g (4096 LSB/g)
        acc1G = 4096;

        // 16.4 dps/lsb scalefactor for all Invensense devices
        gyroScale = 16.4f;
    }

    void checkReboot(bool pendReboot)
    {
        if (pendReboot)
            reset(); // noreturn
    }

    void delayMilliseconds(uint32_t msec)
    {
        delay(msec);
    }

    uint32_t getMicros()
    {
        return micros();
    }

    void ledGreenOff(void)
    {
        digitalWrite(3, LOW);
    }

    void ledGreenOn(void)
    {
        digitalWrite(3, HIGH);
    }

    void ledRedOff(void)
    {
        digitalWrite(4, LOW);
    }

    void ledRedOn(void)
    {
        digitalWrite(4, HIGH);
    }

    bool rcSerialReady(void)
    {
        return rx.frameComplete();
    }

    bool rcUseSerial(void)
    {
        rx.begin();
        return true;
    }

    uint16_t rcReadSerial(uint8_t chan)
    {
        uint8_t chanmap[5] = {1, 2, 3, 0, 5};
        return rx.readRawRC(chanmap[chan]);
    }

    uint16_t rcReadPwm(uint8_t chan)
    {
        (void)chan;
        return 0;
    }

    void reboot(void)
    {
        resetToBootloader();
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

    void writeMotor(uint8_t index, uint16_t value)
    {
        motors[index].setSpeed(value);
    }

    void showArmedStatus(bool armed)
    {
        // XXX this would be a good place to sound a buzzer!
        (void)armed;
    }

    void showAuxStatus(uint8_t status)
    {
        (void)status;
    }

    void extrasInit(class MSP * _msp) 
    {
        (void)_msp;
    }

    void extrasCheckSwitch(void)
    {
    }

    uint8_t extrasGetTaskCount(void)
    {
        return 0;
    }

    bool extrasHandleMSP(uint8_t command)
    {
        (void)command;
        return true;
    }

    void extrasPerformTask(uint8_t taskIndex)
    {
        (void)taskIndex;
    } 

}; // class

} // namespace
