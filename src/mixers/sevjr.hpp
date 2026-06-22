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

            float prop_fl_cw;
            float prop_fr_ccw;
            float prop_r_cw;
            float rudder;

            SevJrMixer() = default;

            SevJrMixer(
                    const float prop_fl_cw,
                    const float prop_fr_ccw,
                    const float prop_r_cw,
                    const float rudder)
                : prop_fl_cw(prop_fl_cw),
                prop_fr_ccw(prop_fr_ccw),
                prop_r_cw(prop_r_cw),
                rudder(rudder) { }

            SevJrMixer& operator=(const SevJrMixer& other) = default;

            static auto Run(const Setpoint & setpoint)-> SevJrMixer
            {
                const auto t = setpoint.thrust;
                const auto r = setpoint.roll;
                const auto p = setpoint.pitch;
                const auto y = setpoint.yaw;

                printf("y=%+3.3f\n", y);

                return SevJrMixer(t+r-p, t-r-p, t+p, 0);
            }
    };

} // namespace hf
