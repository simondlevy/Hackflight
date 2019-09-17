/*
   Common support for angle-based pitch and roll (level-mode) controllers

   Copyright (c) 2018 Juan Gallostra and Simon D. Levy

   Author: Juan Gallostra

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
#include "pidcontroller.hpp"
#include "filters.hpp"

namespace hf {

    class AngleBased {

        private:

            // Simple P controller (no I or D)
            float _Kp = 0;

            float _demandScale = 0;

        public:

            void init(const float Kp, const float demandScale)
            {
                _Kp = Kp;
                _demandScale = demandScale;
            }

            float compute(float demand, float angle)
            {
                float error = demand * _demandScale - angle;

                float pterm = _Kp * error;

                return pterm;
            }

    }; // class AngleBased

} // namespace hf
