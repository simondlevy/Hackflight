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

#include "config.hpp"

namespace hf {

class Board {

    private:

        class MSP * msp;

    protected:

        Config config;

    public: // interface

    //------------------------------------ Core functionality ----------------------------------------------------
        virtual void     init(void) = 0;
        virtual const    Config& getConfig() = 0;
        virtual void     delayMilliseconds(uint32_t msec) = 0;
        virtual void     dump(char * msg) = 0;
        virtual uint64_t getMicros() = 0;
        virtual void     ledSet(uint8_t id, bool is_on, float max_brightness = 255) { (void)id; (void)is_on; (void)max_brightness;}

    //------------------------------------------- IMU -----------------------------------------------------------
        virtual void     imuUpdate(void) { }
        virtual void     imuGetEulerAndGyro(float eulerAnglesRadians[3], int16_t gyroRaw[3]) = 0;

    //-------------------------------------------- RC -----------------------------------------------------
        virtual uint16_t rcReadSerial(uint8_t chan) = 0;
        virtual bool     rcUseSerial(void) = 0;
        virtual uint16_t rcReadPwm(uint8_t chan) = 0;

    //------------------------------------------ Serial ---------------------------------------------------------
        virtual uint8_t  serialAvailableBytes(void) = 0;
        virtual uint8_t  serialReadByte(void) = 0;
        virtual void     serialWriteByte(uint8_t c) = 0;

    //------------------------------------------ Motors ---------------------------------------------------------
        virtual void     writeMotor(uint8_t index, uint16_t value) = 0;

    //------------------------------------------ Extras ---------------------------------------------------------
        virtual void    extrasHandleAuxSwitch(uint8_t auxState) { (void)auxState; }
        virtual uint8_t extrasGetTaskCount(void)  { return 0; }
        virtual bool    extrasHandleMSP(uint8_t command) { (void)command; return false; }
        virtual void    extrasInit(class MSP * _msp) { (void)_msp; }
        virtual void    extrasPerformTask(uint8_t taskIndex) { (void)taskIndex; }
        virtual void    extrasUpdateAccelZ(bool armed) { (void)armed; }

}; // class Board

} // namespace
