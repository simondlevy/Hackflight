/*
   Copyright (c) 2022 Simon D. Levy

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

#include <escs/dshot.h>
#include <dshot_bitbang.h>

class DshotBitbangEsc : public DshotEsc {

    protected:

        virtual escDevice_t * deviceInit(void) override
        {
            return dshotBitbangDevInit(m_motorCount);

            /*
            bbDevice.vTable = bbVTable;
            motorCount = count;
            bbStatus = DSHOT_BITBANG_STATUS_OK;

            memset(bbOutputBuffer, 0, sizeof(bbOutputBuffer));

            for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS &&
                    motorIndex < motorCount; motorIndex++) {
                const timerHardware_t *timerHardware =
                    timerGetConfiguredByTag(ESC_IO_TAGS[motorIndex]);
                const IO_t io = IOGetByTag(ESC_IO_TAGS[motorIndex]);

                uint8_t output = timerHardware->output;
                bbPuPdMode = (output & TIMER_OUTPUT_INVERTED) ?
                    BB_GPIO_PULLDOWN :
                    BB_GPIO_PULLUP;

                if (!IOIsFreeOrPreinit(io)) {
                    // not enough motors initialised for the mixer or a break
                    // in the motors
                    bbDevice.vTable.write = escDevWriteNull;
                    bbDevice.vTable.updateStart = escUpdateStartNull;
                    bbDevice.vTable.updateComplete = escUpdateCompleteNull;
                    bbStatus = DSHOT_BITBANG_STATUS_MOTOR_PIN_CONFLICT;
                    return NULL;
                }

                int pinIndex = IO_GPIOPinIdx(io);

                bbMotors[motorIndex].pinIndex = pinIndex;
                bbMotors[motorIndex].io = io;
                bbMotors[motorIndex].output = output;
                bbMotors[motorIndex].iocfg = IO_CONFIG(GPIO_Mode_OUT, GPIO_Speed_50MHz,
                        GPIO_OType_PP, bbPuPdMode);

                IOInit(io, OWNER_MOTOR, RESOURCE_INDEX(motorIndex));
                IOConfigGPIO(io, bbMotors[motorIndex].iocfg);
                if (output & TIMER_OUTPUT_INVERTED) {
                    IOLo(io);
                } else {
                    IOHi(io);
                }

                // Fill in motors structure for 4way access 
                motors[motorIndex].io = bbMotors[motorIndex].io;
            }

            return &bbDevice;
            */
        }        

public:

    DshotBitbangEsc(uint8_t count) 
: DshotEsc(count)
{
}


}; // class DshotBitbangEsc
