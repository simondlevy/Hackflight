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

            SBUS sbus = SBUS(Serial1);

            float _chanvals[16] = {};

            void sendChannelValues(void)
            {
                sbus.writeCal(_chanvals);
            }

        protected:

            virtual void setChannelValues(demands_t & demands) override
            {
                _chanvals[0] = demands.throttle;
                _chanvals[1] = demands.roll;
                _chanvals[2] = demands.pitch;
                _chanvals[3] = demands.yaw;

                _chanvals[4] = +1.0;

                sendChannelValues();
            }

            virtual void sendDisarmed(void) override
            {
                _chanvals[4] = -1.0; // Aux1

                sendChannelValues();
            }

        public:

            void begin(void)
            {
                memset(_chanvals, 0, 16*sizeof(float));

                sbus.begin();
            }

    }; // class SbusProxy

} // namespace hf
