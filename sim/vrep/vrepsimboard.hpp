/*
   Hackflight Board class declaration for V-REP simulator

   Copyright (C) Simon D. Levy, Matt Lubas, and Julio Hidalgo Lopez 2016

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

#include <hackflight.hpp>
#include <board.hpp>
#include <config.hpp>

namespace hf {

    class VrepSimBoard : public Board {

        public:

            virtual void     imuInit(void) override; 
            virtual void     imuGetEulerAngles(float eulerAnglesRadians[3]) override;
            virtual void     init(void) override;
            virtual const    Config& getConfig() override;
            virtual void     imuGetGyro(int16_t gyroADC[3]) override;
            virtual void     ledSet(uint8_t id, bool is_on, float max_brightness)  override;
            virtual uint64_t getMicros() override;
            virtual bool     rcUseSerial(void) override;
            virtual uint16_t rcReadPwm(uint8_t chan) override;
            virtual uint8_t  serialAvailableBytes(void) override;
            virtual uint8_t  serialReadByte(void) override;
            virtual void     serialWriteByte(uint8_t c) override;
            virtual void     dump(char * msg) override;
            virtual void     writeMotor(uint8_t index, uint16_t value) override;
            virtual void     extrasCheckSwitch(uint8_t status) override;
            virtual uint8_t  extrasGetTaskCount(void) override;
            virtual bool     extrasHandleMSP(uint8_t command) override;
            virtual void     extrasInit(class MSP * _msp) override;
            virtual void     extrasPerformTask(uint8_t taskIndex) override;
            virtual uint16_t rcReadSerial(uint8_t chan) override;
            virtual void     delayMilliseconds(uint32_t msec) override;

    private: // types

        typedef struct stdev_t {
            float m_oldM, m_newM, m_oldS, m_newS;
            int m_n;
        } stdev_t;

    private: //methods

        static void normalizeV(float src[3], float dest[3]);

    private: // fields

        float       EstG[3];
        float       EstN[3];
        float       gyroCmpfFactor;
        float       gyroScale;

    };  // class VrepSimBoard

}  // namespace hf
