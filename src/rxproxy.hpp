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

#include "hackflightlite.hpp"

namespace hf {

    class RXProxy : protected Demander {

        friend class HackflightLite;

        private:

            // Helps us detect change in armed status
            bool _wasArmed = false;

        protected:

            RXProxy(void)
            {
                _wasArmed = false;
            }

            void checkArmDisarm(bool armed)
            {
                if (armed) {
                    if (!_wasArmed) {
                        setArmedStatus(true);
                    }
                    _wasArmed = true;
                }
                else {
                    if (_wasArmed) {
                        setArmedStatus(false);
                    }
                    _wasArmed = false;
                }
            }

            void run(demands_t demands) override
            {
                setDemands(demands);
            }

            void cut(void) override
            {
                demands_t demands;
                demands.throttle = -1;
                setDemands(demands);
            }

            virtual void setDemands(demands_t & demands) = 0;

            virtual void setArmedStatus(bool status) = 0;

    }; // class RXProxy

} // namespace hf
