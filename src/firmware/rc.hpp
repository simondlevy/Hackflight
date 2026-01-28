/**
 * Copyright (C) 2025 Simon D. Levy
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

#include <string.h>

#include <firmware/comms.hpp>
#include <msp/__messages__.h>
#include <msp/parser.hpp>
#include <firmware/timer.hpp>

namespace hf {

    class RC {

        public:

            typedef struct {

                uint32_t timestamp;
                bool armed;
                bool hovering;
                demands_t demands;

            } setpoint_t;

            static void getSetpoint(const uint32_t tick, setpoint_t & setpoint)
            {
                static Timer _timer;

                MspParser parser = {};

                if (_timer.ready(FREQ_HZ)) {

                    uint8_t byte = 0;

                    while (Comms::read_byte(&byte)) {

                        switch (parser.parse(byte)) {

                            case MSP_SET_ARMING:
                                setpoint.armed = !setpoint.armed;
                                setpoint.timestamp = tick;
                                break;

                            case MSP_SET_IDLE:
                                setpoint.hovering = false;
                                setpoint.timestamp = tick;
                                break;

                            case MSP_SET_HOVER:
                                setpoint.hovering = true;
                                setpoint.demands.thrust = parser.getFloat(0);
                                setpoint.demands.pitch = parser.getFloat(1);
                                setpoint.demands.roll = parser.getFloat(2);
                                setpoint.demands.yaw = parser.getFloat(3);
                                setpoint.timestamp = tick;
                                break;

                            default:
                                break;
                        }
                    }
                }
            }

        private:

            static constexpr float FREQ_HZ = 100;
    };

}
