/**
 * Copyright (C) 2024 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "clock.hpp"
#include "datatypes.h"

class ClosedLoopController {

    public:

        /**
          * A closed-loop controller accepts a state vector and a set of
          * demands and modifies the demands.
          */
        virtual void run(
                const vehicleState_t & state, demands_t & demands) = 0;

    protected:

        Clock::rate_t _updateRate;

        float _dt;

        void init(const Clock::rate_t updateRate)
        {
            _updateRate = updateRate;

            _dt = 1. / updateRate;
        }
};
