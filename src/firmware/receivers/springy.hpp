/**
 * Class for R/C receivers using spring-throttle transmitter
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

    class SpringyReceiver {

        public:

            Setpoint axes;
            bool is_armed;
            bool is_hovering;
            uint32_t timestamp_msec;

            SpringyReceiver() = default;

            SpringyReceiver(
                    const Setpoint & axes,
                    const bool is_armed,
                    const bool is_hovering,
                    const uint32_t timestamp_msec,
                    const uint16_t aux1)
                :
                    axes(axes),
                    is_armed(is_armed),
                    is_hovering(is_hovering),
                    timestamp_msec(timestamp_msec),
                    _aux1(aux1) {}

            SpringyReceiver& operator=(
                    const SpringyReceiver& other) = default;

            static auto update(
                    const SpringyReceiver & data,
                    const uint16_t throttle,
                    const uint16_t roll,
                    const uint16_t pitch,
                    const uint16_t yaw,
                    const uint16_t aux1,
                    const uint16_t aux2,
                    const uint32_t msec_curr) -> SpringyReceiver
            {
                const auto axes = Setpoint(
                        scale(throttle),
                        scale(roll),
                        scale(pitch),
                        scale(yaw));

                // Push-button arming; ignores startup transient
                const auto did_aux1_change =
                    data._aux1 >= 988 && aux1 != data._aux1;

                const auto is_hovering = false;

                const auto is_armed = 
                    did_aux1_change ? !data.is_armed : data.is_armed;

                return SpringyReceiver(axes, is_armed, is_hovering, msec_curr,
                        aux1);
            }

            static void report(const SpringyReceiver & data)
            {
                static uint32_t _count;

                const auto ax = data.axes;

                printf("%5lu | t=%+3.3f r=%+3.3f p=%3.3f y=%+3.3f | "
                        "armed=%d hover=%d\n",
                        _count++, ax.thrust, ax.roll, ax.pitch, ax.yaw,
                        data.is_armed, data.is_hovering);
            }

        private:

            uint16_t _aux1;

            static auto scale(const uint16_t val) -> float
            {
                return 2 * (val - 1500.f) / 1024;
            }

    };
}
