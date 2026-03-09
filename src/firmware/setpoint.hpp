/*
   Based on  https://github.com/nickrehm/dRehmFlight

   Copyright (C) 2026 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#include <hackflight.h>
#include <datatypes.hpp>
#include <pidcontrol/pids/position.hpp>

namespace hf {

    Setpoint mksetpoint(const float * rx_chanvals)
    {
        return Setpoint(
                (rx_chanvals[0]+1)/2,
                rx_chanvals[1] * PositionController::MAX_DEMAND_DEG, 
                rx_chanvals[2] * PositionController::MAX_DEMAND_DEG, 
                rx_chanvals[3]);
    }
}
