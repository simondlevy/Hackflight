/*
   rangelndflow.hpp : MSP-based sensor using rangefinder and optial flow

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

#include "sensors/mspsensor.hpp"
#include "debug.hpp"

namespace hf {

    class RangeAndFlow : public MspSensor {

        virtual bool ready(float time) override
        {
            (void)time;
            return false;
        }

        virtual void modifyState(state_t & state, float time)  override
        {
            (void)state;
            (void)time;
        }

        virtual void handle_SET_RANGE_AND_FLOW(int16_t  range, int16_t  flowx, int16_t  flowy) override
        {
            hf::Debug::printf("%04d %+3d %+3d\n", range, flowx, flowy); 
        }

    };  // class RangeAndFlow

} // namespace
