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

#include <firmware/receivers/traditional.hpp>

namespace hf {

    class SpringyReceiver {

        public:

            Setpoint setpoint;
            bool requested_arming;
            bool requested_hover;
            uint32_t timestamp_msec;

            SpringyReceiver() = default;

            SpringyReceiver(
                    const Setpoint & setpoint,
                    const bool requested_arming,
                    const bool requested_hover,
                    const uint32_t timestamp_msec,
                    const uint16_t aux1,
                    const uint16_t aux2) :
                setpoint(setpoint),
                requested_arming(requested_arming),
                requested_hover(requested_hover),
                timestamp_msec(timestamp_msec),
                _aux2(aux2)

            {
                _traditional = TraditionalReceiver(
                        setpoint, requested_arming, true, timestamp_msec, aux1);
            }

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
                const auto traditional = TraditionalReceiver::update(
                        data._traditional, throttle, roll, pitch, yaw, aux1,
                        msec_curr, false);
                        
                const auto requested_hover = aux2 > 1500;

                return SpringyReceiver(traditional.setpoint,
                        traditional.requested_arming, requested_hover,
                        traditional.timestamp_msec, aux1, aux2);
            }

        private:

            uint16_t _aux2;

            TraditionalReceiver _traditional;

    };
}
