/*
   pidcontroller.hpp : Abstract class for PID controllers

   Copyright (c) 2018 Simon D. Levy

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

#include "datatypes.hpp"

namespace hf {

    class PidController {

        friend class Hackflight;

        protected:

        virtual bool modifyDemands(state_t & state, demands_t & demands, float currentTime) = 0;

        virtual bool shouldFlashLed(void) { return false; }

        virtual void updateReceiver(demands_t & demands, bool throttleIsDown) { (void)demands; (void)throttleIsDown; }

        uint8_t auxState = 0;

    };  // class PidController

} // namespace hf
