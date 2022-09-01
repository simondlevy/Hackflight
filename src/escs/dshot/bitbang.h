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

        virtual void deviceInit(void) override
        {
            static const uint8_t pins[4] = {32, 33, 19, 18};

            dshotBitbangDevInit(pins, m_motorCount);
        }        

        virtual bool enable(void) override
        {
            return bbEnableMotors();
        }

        virtual void postInit(void) override
        {
            bbPostInit(m_protocol);
        }

        virtual void updateComplete(void)override
        {
            bbUpdateComplete(m_motorCount);
        }

        virtual bool updateStart(void) override
        {
            return bbUpdateStart();
        }

        virtual void write(uint8_t index, float value) override
        {
            bbWrite(index, value);
        }

        virtual void writeInt(uint8_t index, uint16_t value) override
        {
            bbWrite(index, value);
        }

    public:

        DshotBitbangEsc(uint8_t count, dshotProtocol_t protocol=DSHOT600) 
            : DshotEsc(count, protocol)
        {
        }


}; // class DshotBitbangEsc
