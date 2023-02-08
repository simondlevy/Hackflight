/*
   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under the
   terms of the GNU General Public License as published by the Free Software
   Foundation, either version 3 of the License, or (at your option) any later
   version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
   WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
   PARTICULAR PURPOSE. See the GNU General Public License for more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <Wire.h>

#include <USFS.h>

#include "board/stm32.h"
#include "esc/brushed.h"
#include "imu/ladybug.h"

class LadybugBoard : public Stm32Board {

    private:

        // Arbitrary; unused
        static const uint8_t  ACCEL_BANDWIDTH  = 3;
        static const uint8_t  GYRO_BANDWIDTH   = 3;
        static const uint8_t  QUAT_DIVISOR     = 1;
        static const uint8_t  MAG_RATE         = 100;
        static const uint8_t  ACCEL_RATE_TENTH = 20; // Multiply by 10 to get actual rate
        static const uint8_t  BARO_RATE        = 50;
        static const uint16_t ACCEL_SCALE      = 8;
        static const uint16_t MAG_SCALE        = 1000;

        static const uint8_t  GYRO_RATE_TENTH = 100;   // 1/10th actual rate

        static const uint8_t IMU_INTERRUPT_PIN = 0x0C;

        static const uint8_t INTERRUPT_ENABLE = Usfs::INTERRUPT_RESET_REQUIRED |
            Usfs::INTERRUPT_ERROR |
            Usfs::INTERRUPT_GYRO | 
            Usfs::INTERRUPT_QUAT;

        std::vector<uint8_t> motorPins = {0x0D, 0x10, 0x03, 0x0B};

        Usfs m_usfs;

        LadybugImu m_imu; 

        BrushedEsc esc = BrushedEsc(motorPins);

        int16_t m_rawAccel[3] = {};

    public:

        static const uint8_t LED_PIN = 0x12;

        LadybugBoard(std::vector<PidController *> & pids, Mixer & mixer)
            : Stm32Board(&m_imu, pids, mixer, esc, -LED_PIN) // note inverted LED pin
        {
        }

        void begin(void (*isr)(void))
        {
            Serial.begin(115200);

            Wire.begin();
            Wire.setClock(400000); 
            delay(100);

            setImuInterrupt(IMU_INTERRUPT_PIN, isr, RISING);  

            Board::begin();

            m_usfs.loadFirmware(); 

            m_usfs.begin(
                    ACCEL_BANDWIDTH,
                    GYRO_BANDWIDTH,
                    QUAT_DIVISOR,
                    MAG_RATE,
                    ACCEL_RATE_TENTH,
                    GYRO_RATE_TENTH,
                    BARO_RATE,
                    INTERRUPT_ENABLE);

            // Clear interrupts
            Usfs::checkStatus();

        }

        void step(void)
        {
            static int16_t rawGyro[3];

            if (m_imu.gotNewData) { 

                m_imu.gotNewData = false;  

                uint8_t eventStatus = Usfs::checkStatus(); 

                if (Usfs::eventStatusIsError(eventStatus)) { 
                    Usfs::reportError(eventStatus);
                }

                if (Usfs::eventStatusIsGyrometer(eventStatus)) { 
                    m_usfs.readGyrometerRaw(rawGyro);
                }

                if (Usfs::eventStatusIsQuaternion(eventStatus)) { 
                    m_usfs.readQuaternion(m_imu.qw, m_imu.qx, m_imu.qy, m_imu.qz);
                }
            } 

            Board::step(rawGyro, m_rawAccel);
        }

}; // class LadybugBoard
