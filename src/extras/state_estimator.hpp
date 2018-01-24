/* 
    state_estimator.hpp: Abstract class for state estimation

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
    along with EM7180.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "debug.hpp"
#include "datatypes.hpp"
#include "timer.hpp"

namespace hf {

    class StateEstimator {

        public:

            virtual void estimate(vehicle_state_t & state)  = 0;

    }; // class StateEstimator

} // namespace hf
