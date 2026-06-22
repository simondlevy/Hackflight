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

    class Mixer {

        public:

            float m_rr_cw;
            float m_rf_ccw;
            float m_lr_ccw;
            float m_lf_cw;

            Mixer() = default;

            Mixer(
                    const float m_rr_cw,
                    const float m_rf_ccw,
                    const float m_lr_ccw,
                    const float m_lf_cw)
                : m_rr_cw(m_rr_cw),
                m_rf_ccw(m_rf_ccw),
                m_lr_ccw(m_lr_ccw),
                m_lf_cw(m_lf_cw) { }

            Mixer& operator=(const Mixer& other) = default;

            static auto Run(const Setpoint & setpoint)-> Mixer
            {
                const auto t = setpoint.thrust;
                const auto r = setpoint.roll;
                const auto p = setpoint.pitch;
                const auto y = setpoint.yaw;

                const float m_rr_cw = t - r + p - y;
                const float m_rf_ccw = t - r - p + y;
                const float ml_lr_ccw = t + r + p + y;
                const float m_lf_cw = t + r - p - y;

                return Mixer(m_rr_cw, m_rf_ccw, ml_lr_ccw, m_lf_cw);
            }
    };

} // namespace hf
