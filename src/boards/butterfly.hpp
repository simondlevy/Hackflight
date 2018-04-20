/*
   butterfly.hpp : Implementation of Hackflight Board routines for Butterfly
                   dev board + MPU9250 IMU

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
#include <MPU9250.h> 

#include "hackflight.hpp"
#include "realboard.hpp"

namespace hf {

    class Butterfly : public RealBoard {

        private:

            MPU9250  imu;

            const uint8_t LED_PIN       = 13;
            const uint8_t INTERRUPT_PIN = 8;

            const uint8_t Ascale     = AFS_2G;
            const uint8_t Gscale     = GFS_250DPS;
            const uint8_t Mscale     = MFS_16BITS;
            const uint8_t Mmode      = M_100Hz;
            const uint8_t sampleRate = 0x04;         

            float aRes;
            float gRes;
            float mRes;

            // XXX we should be loading these values from pre-calibrated data
            float gyroBias[3]        = {0,0,0};
            float accelBias[3]       = {0,0,0};
            float magBias[3]         = {0,0,0};
            float magScale[3]        = {1,1,1};      
            float magCalibration[3]  = {1,1,1};      

        protected:

            void init(void)
            {
                // Begin serial comms
                Serial.begin(115200);

                // Setup LED_PINs and turn them off
                pinMode(LED_PIN, OUTPUT);
                digitalWrite(LED_PIN, LOW);

                // Start I^2C
                Wire.begin();
                Wire.setClock(400000); // I2C frequency at 400 kHz
                delay(1000);

                // Reset the MPU9250
                imu.resetMPU9250(); 

                // get sensor resolutions, only need to do this once
                aRes = imu.getAres(Ascale);
                gRes = imu.getGres(Gscale);
                mRes = imu.getMres(Mscale);

                imu.initMPU9250(Ascale, Gscale, sampleRate); 

                // Get magnetometer calibration from AK8963 ROM
                imu.initAK8963(Mscale, Mmode, magCalibration);

                // Do general real-board initialization
                RealBoard::init();
            }

            void delayMilliseconds(uint32_t msec)
            {
                delay(msec);
            }

            void ledSet(bool is_on)
            { 
                digitalWrite(LED_PIN, is_on ? HIGH : LOW);
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
                (void)index;
                (void)value;
            }

            bool getGyrometer(float gyro[3])
            {

                if(imu.checkNewAccelGyroData()) {

                    Debug::printf("%d\n", millis());

                    return true;
                }

                return false;
            }

            bool getQuaternion(float quat[4])
            {
                // Fake it for now
                quat[0] = 0.3f;
                quat[1] = 0.0f;
                quat[2] = 0.0f;
                quat[3] = 1.0f;

                return true;
            }

            bool getAccelerometer(float accelGs[3])
            {
                (void)accelGs;
                return false;
    }

    bool getBarometer(float & pressure)
    {
        (void)pressure;
        return false;
    }

}; // class Butterfly

void Board::outbuf(char * buf)
{
    Serial.print(buf);
}

} // namespace hf
