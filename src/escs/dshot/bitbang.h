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

    private:

        typedef enum {
            MODE_OFF,
            MODE_ON,
            MODE_AUTO,
        } mode_e;

        typedef enum {
            STATUS_OK,
            STATUS_MOTOR_PIN_CONFLICT,
            STATUS_NO_PACER,
            STATUS_TOO_MANY_PORTS,
        } status_e;

        typedef struct {
            dshotProtocolControl_t protocolControl;
            int pinIndex;    
            int portIndex;
            IO_t io; 
            uint8_t output;
            uint32_t iocfg;
            bbPort_t *bbPort;
            bool configured;
            bool enabled;
        } bbMotor_t;

    protected:

        virtual void deviceInit(void) override
        {
            dshotBitbangDevInit(m_motorPins, m_motorCount);
        }        

        virtual bool enable(void) override
        {
            return dshotBitbangEnableMotors();
        }

        virtual void postInit(void) override
        {
            dshotBitbangPostInit(m_protocol);
        }

        virtual void updateComplete(void)override
        {
            dshotBitbangUpdateComplete(m_motorCount);
        }

        virtual bool updateStart(void) override
        {
            return dshotBitbangUpdateStart();
        }

        virtual void write(uint8_t index, float value) override
        {
            dshotBitbangWrite(index, value);
        }

        virtual void writeInt(uint8_t index, uint16_t value) override
        {
            dshotBitbangWrite(index, value);
        }

    public:

        DshotBitbangEsc(vector<uint8_t> * pins, dshotProtocol_t protocol=DSHOT600) 
            : DshotEsc(pins, protocol)
        {
        }


}; // class DshotBitbangEsc
