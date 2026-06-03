/**
 * Class for old-school R/C receiver data
 *
 * Copyright (C) 2026 Simon D. Levy
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

#include <datatypes.hpp>

namespace hf {

    class TraditionalReceiver {

        private:

            static constexpr float THROTTLE_DOWN_MAX = -0.95;

        public:

            Setpoint axes;
            bool is_armed;
            bool is_throttle_down;
            uint16_t aux;
            uint32_t timestamp_msec;

            TraditionalReceiver() = default;

            TraditionalReceiver(
                    const Setpoint & axes,
                    const bool is_armed,
                    const bool is_throttle_down,
                    const uint16_t aux,
                    const uint32_t timestamp_msec)
                :
                    axes(axes),
                    is_armed(is_armed),
                    is_throttle_down(is_throttle_down),
                    aux(aux),
                    timestamp_msec(timestamp_msec) {}

            TraditionalReceiver& operator=(
                    const TraditionalReceiver& other) = default;

            static auto update(
                    const TraditionalReceiver & data,
                    const uint16_t throttle,
                    const uint16_t roll,
                    const uint16_t pitch,
                    const uint16_t yaw,
                    const uint16_t aux,
                    const uint32_t msec_curr) -> TraditionalReceiver
            {
                const auto axes = Setpoint(
                        scale(throttle),
                        scale(roll),
                        scale(pitch),
                        scale(yaw));

                const auto is_throttle_down = axes.thrust <
                    THROTTLE_DOWN_MAX;

                // Push-button arming; ignores startup transient
                const auto did_aux_change = data.aux >= 988 && aux !=
                    data.aux;

                const auto is_armed = 
                    did_aux_change && data.is_armed ? false :
                    did_aux_change && data.is_throttle_down ? true :
                    data.is_armed;

                return TraditionalReceiver(axes, is_armed,
                        is_throttle_down, aux, msec_curr); }

            static void report(const TraditionalReceiver & data)
            {
                static uint32_t _count;

                const auto ax = data.axes;

                printf("%5lu | t=%+3.3f r=%+3.3f p=%3.3f y=%+3.3f\n",
                        _count++, ax.thrust, ax.roll, ax.pitch, ax.yaw);
            }

        private:

            static auto scale(const uint16_t val) -> float
            {
                return 2 * (val - 1500.f) / 1024;
            }

    };
}
