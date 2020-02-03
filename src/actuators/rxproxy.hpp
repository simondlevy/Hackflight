/*
   RXProxy class

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

namespace hf {

    class RXProxy : protected Actuator {

        friend class Hackflight;

            void run(demands_t demands) override
            {
                setChannelValues(demands);
            }

            void cut(void) override
            {
                demands_t demands = {};
                demands.throttle = -1;
                setChannelValues(demands);
            }

            virtual void setChannelValues(demands_t & demands) = 0;

            virtual void sendDisarmed(void) = 0;

    }; // class RXProxy

} // namespace hf
