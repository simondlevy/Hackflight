/*
 *  QuadX motor mixer for Hackflight
 *
 *                 cw  ccw
 *                   \ /
 *                    X 
 *                   / \
 *                 ccw  cw
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

#include <datatypes.hpp>

namespace hf {

    class QuadXMixer {

        public:

            float rr_cw;
            float rf_ccw;
            float lr_ccw;
            float lf_cw;

            QuadXMixer() = default;

            QuadXMixer(
                    const float rr_cw,
                    const float rf_ccw,
                    const float lr_ccw,
                    const float lf_cw)
                : rr_cw(rr_cw),
                rf_ccw(rf_ccw),
                lr_ccw(lr_ccw),
                lf_cw(lf_cw) { }

            QuadXMixer& operator=(const QuadXMixer& other) = default;

            static auto Run(const Setpoint & setpoint)-> QuadXMixer
            {
                const auto t = setpoint.thrust;
                const auto r = setpoint.roll;
                const auto p = setpoint.pitch;
                const auto y = setpoint.yaw;

                return QuadXMixer(
                        t - r + p - y,
                        t - r - p + y,
                        t + r + p + y,
                        t + r - p - y);
            }
    };

} // namespace hf
