/*
 *  SEV Jr motor mixer for Hackflight
 *
 *                 cw  ccw
 *                   \ /
 *                    X 
 *                    |
 *                cw / rudder
 *             
 *
 * Copyright (C) 2024 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */


#pragma once

#include <stdio.h>

#include <datatypes.hpp>

namespace hf {

    class SevJrMixer {

        public:

            float fl_cw;
            float fr_ccw;
            float r_cw;

            SevJrMixer() = default;

            SevJrMixer( const float fl_cw, const float fr_ccw, const float r_cw)
                : fl_cw(fl_cw), fr_ccw(fr_ccw), r_cw(r_cw) { }

            SevJrMixer& operator=(const SevJrMixer& other) = default;

            static auto Run(const Setpoint & setpoint)-> SevJrMixer
            {
                const auto t = setpoint.thrust;
                const auto r = setpoint.roll;
                const auto p = setpoint.pitch;
                // const auto y = setpoint.yaw;

                return SevJrMixer(t + r - p, t - r - p, t + p);
            }
    };

} // namespace hf
