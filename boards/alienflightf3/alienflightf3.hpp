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

        virtual void extrasCheckSwitch(void) override;
        virtual bool extrasHandleMSP(uint8_t command) override;
        virtual void extrasInit(class MSP * _msp) override;
        virtual void extrasPerformTask(uint8_t taskIndex) override;

        virtual void     delayMilliseconds(uint32_t msec) override;
        virtual uint32_t getMicros() override;

        virtual void     dump(char * msg) override;

        virtual void     ledGreenOff(void) override;
        virtual void     ledGreenOn(void) override;
        virtual void     ledRedOff(void) override;
        virtual void     ledRedOn(void) override;

    };


} // namespace
