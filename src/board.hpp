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
        virtual void     debug(char * msg) = 0;
        virtual void     imuGetEuler(float eulerAnglesRadians[3]) = 0;
        virtual void     imuGetGyro(int16_t gyroRaw[3]) = 0;
        virtual uint64_t getMicros() = 0;
        virtual void     writeMotor(uint8_t index, uint16_t value) = 0;

    //------------------------------------------ LED ------------------------------------------------------------
        virtual void     ledSet(uint8_t id, bool is_on) { (void)id; (void)is_on; }

    //------------------------------------------ Serial ---------------------------------------------------------
        virtual uint8_t  serialAvailableBytes(void) { return 0; }
        virtual uint8_t  serialReadByte(void)  { return 0; }
        virtual void     serialWriteByte(uint8_t c) { (void)c; }

    //------------------------------------------ Simulation  ----------------------------------------------------
        virtual bool     skipArming(void) { return false; }

    //------------------------------------------ Extras ---------------------------------------------------------
        virtual bool    extrasHaveBaro(void) { return false; }
        virtual float   extrasGetBaroPressure(void) { return 0; }
        virtual void    extrasImuGetAccel(int16_t accelRaw[3]) { (void)accelRaw; }
        virtual void    extrasImuPoll(void) { }

}; // class Board

} // namespace
