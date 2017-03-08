/*
   board.hpp : class header for board-specific routines

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


namespace hf {

class Board {

    private:

        class MSP * msp;

    public:

        // your implementation should support these methods

        // Core functionality
        static void     delayMilliseconds(uint32_t msec);
        static void     dump(char * msg);
        static uint32_t getMicros();
        static void     imuInit();
        static void     imuRead(int16_t accADC[3], int16_t gyroADC[3]);
        static void     init(uint16_t & acc1G, float & gyroScale,
                             uint32_t & imuLooptimeUsec, uint32_t & calibratingGyroMsec);

        virtual void     ledGreenOff(void) = 0;
        virtual void     ledGreenOn(void) = 0;
        virtual void     ledRedOff(void) = 0;
        virtual void     ledRedOn(void) = 0;

        static uint16_t rcReadSerial(uint8_t chan);
        static bool     rcSerialReady(void);
        static bool     rcUseSerial(void);
        static uint16_t readPWM(uint8_t chan);
        static uint8_t  serialAvailableBytes(void);
        static uint8_t  serialReadByte(void);
        static void     serialWriteByte(uint8_t c);
        static void     writeMotor(uint8_t index, uint16_t value);

        // extra functionality
        void            extrasCheckSwitch(void);
        static uint8_t  extrasGetTaskCount(void);
        bool            extrasHandleMSP(uint8_t command);
        void            extrasInit(class MSP * _msp);
        void            extrasPerformTask(uint8_t taskIndex);

        // helps with simulation
        static void     showArmedStatus(bool armed);
        static void     showAuxStatus(uint8_t status);

        // STM32
        virtual void     checkReboot(bool pendReboot) = 0;
        virtual void     reboot(void) = 0;

        // default constants
        static const uint32_t DEFAULT_IMU_LOOPTIME_USEC     = 3500;
        static const uint32_t DEFAULT_GYRO_CALIBRATION_MSEC = 3500;

}; // class Board

} // namespace
