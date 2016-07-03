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

#ifdef __arm__
extern "C" {
#endif

    class Board {

        private:

            // add stuff as needed here

        public:

            // your implementation should support these methods

            void     init(uint32_t & imuLooptimeUsec, uint32_t & calibratingGyroMsec);

            bool     baroInit(void);
            void     baroUpdate(void);
            int32_t  baroGetPressure(void);
            void     checkReboot(bool pendReboot);
            void     delayMilliseconds(uint32_t msec);
            uint32_t getMicros();
            void     imuInit(uint16_t & acc1G, float & gyroScale);
            void     imuRead(int16_t accADC[3], int16_t gyroADC[3]);
            void     ledGreenOff(void);
            void     ledGreenOn(void);
            void     ledGreenToggle(void);
            void     ledRedOff(void);
            void     ledRedOn(void);
            void     ledRedToggle(void);
            uint16_t readPWM(uint8_t chan);
            void     reboot(void);
            uint8_t  serialAvailableBytes(void);
            uint8_t  serialReadByte(void);
            void     serialWriteByte(uint8_t c);
            void     writeMotor(uint8_t index, uint16_t value);

    }; // class Board


#ifdef __arm__
} // extern "C"
#endif
