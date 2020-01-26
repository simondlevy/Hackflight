/*
   RXProxy class for SBUS

   Copyright (c) 2020 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MEReceiverHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "rxproxy.hpp"
#include "SBUS.h"

namespace hf {

    class SbusProxy : public RXProxy {

        private:

            static const uint16_t MINVAL = 172;
            static const uint16_t MAXVAL = 1811;

            SBUS sbus = SBUS(Serial1);

            uint16_t _channels[16];

            static uint16_t val2int(float val)
            {
                return MINVAL + (uint16_t)(val * (MAXVAL-MINVAL));
            }

        protected:

            virtual void setChannelValues(demands_t & demands, bool armed) override
            {

                _channels[0] = val2int(demands.throttle);
                _channels[1] = val2int(demands.roll);
                _channels[2] = val2int(demands.pitch);
                _channels[3] = val2int(demands.yaw);

                _channels[4] = armed ? MAXVAL : MINVAL; // Aux1
            }

        public:

            void begin(void)
            {
                sbus.begin();

                // Set channels to neutral values
                for (uint8_t k=0; k<16; ++k) {
                    _channels[k] = (MINVAL + MAXVAL) / 2;
                }
            }

    }; // class SbusProxy

} // namespace hf
