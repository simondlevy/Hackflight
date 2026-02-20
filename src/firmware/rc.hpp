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
                setpoint_t setpoint;

            } message_t;

            static void getSetpoint(const uint32_t tick, message_t & message)
            {
                static Timer _timer;

                MspParser parser = {};

                if (_timer.ready(FREQ_HZ)) {

                    uint8_t byte = 0;

                    while (Comms::read_byte(&byte)) {

                        switch (parser.parse(byte)) {

                            case MSP_SET_ARMING:
                                message.armed = !message.armed;
                                message.timestamp = tick;
                                break;

                            case MSP_SET_IDLE:
                                message.hovering = false;
                                message.timestamp = tick;
                                break;

                            case MSP_SET_HOVER:
                                message.hovering = true;
                                message.setpoint.thrust = parser.getFloat(0);
                                message.setpoint.pitch = parser.getFloat(1);
                                message.setpoint.roll = parser.getFloat(2);
                                message.setpoint.yaw = parser.getFloat(3);
                                message.timestamp = tick;
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
