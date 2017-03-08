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

#include "board.hpp"
#include "hackflight.hpp"

namespace hf {

    class AlienflightF3 : public Board {

        virtual uint8_t  serialAvailableBytes(void) override;
        virtual uint8_t  serialReadByte(void) override;
        virtual void     serialWriteByte(uint8_t c) override;

        virtual void     dump(char * msg) override;

        virtual void     delayMilliseconds(uint32_t msec) override;
        virtual uint32_t getMicros() override;

        virtual uint16_t rcReadSerial(uint8_t chan) override;
        virtual bool     rcSerialReady(void) override;
        virtual bool     rcUseSerial(void) override;
        virtual uint16_t rcReadPwm(uint8_t chan) override;

        virtual void     ledGreenOff(void) override;
        virtual void     ledGreenOn(void) override;
        virtual void     ledRedOff(void) override;
        virtual void     ledRedOn(void) override;

        virtual void     checkReboot(bool pendReboot) override;
        virtual void     reboot(void) override;

        virtual void     showArmedStatus(bool armed) override;
        virtual void     showAuxStatus(uint8_t status) override;

        virtual void    extrasCheckSwitch(void) override;
        virtual uint8_t extrasGetTaskCount(void) override;
        virtual bool    extrasHandleMSP(uint8_t command) override;
        virtual void    extrasInit(class MSP * _msp) override;
        virtual void    extrasPerformTask(uint8_t taskIndex) override;

        virtual void    writeMotor(uint8_t index, uint16_t value) override;
    };

} // namespace
