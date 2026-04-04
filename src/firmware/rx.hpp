/**
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

    class RX {

        public:

            class Data {

                public:

                    Setpoint axes;
                    float aux;
                    bool is_armed;
                    bool is_throttle_down;

                    Data() = default;

                    Data(
                            const Setpoint & axes,
                            const float aux,
                            const bool is_armed,
                            const bool is_throttle_down)
                        :
                            axes(axes),
                            aux(aux),
                            is_armed(is_armed),
                            is_throttle_down(is_throttle_down) {}

                    Data& operator=(const Data& other) = default;
            };

            RX(HardwareSerial * serial) : _serial(serial) { }

            void begin();

            auto read() -> Data;

        private:

            HardwareSerial * _serial;
    };
}
