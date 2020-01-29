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

        protected:

            virtual void setChannelValues(demands_t & demands) override
            {
            }

        public:

            void begin(void)
            {
                //sbus.begin();
            }

    }; // class SbusProxy

} // namespace hf
