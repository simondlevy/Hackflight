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

#include "demands.h"
#include "utils.h"
#include "vstate.h"

class PidController {

    private:

        static const uint32_t FREQ_HZ = 8000;

    protected:

         virtual auto getDemands(
                const int32_t dusec,
                const Demands & demands,
                const VehicleState & vstate,
                const bool reset) -> Demands = 0;

    public:

        static const uint32_t PERIOD = 1000000 / FREQ_HZ;

        static constexpr float DT = PERIOD * 1e-6f;

         auto update(
                const uint32_t usec,
                const Demands & demands,
                const VehicleState & vstate,
                const bool reset) -> Demands 
         {
             static uint32_t _prev;

             const auto dusec = intcmp(usec, _prev);

             _prev = usec;

             return getDemands(dusec, demands, vstate, reset);
         }

};
