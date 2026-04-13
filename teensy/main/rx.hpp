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

#include <Arduino.h>

#include <datatypes.hpp>
#include <firmware/rxdata.hpp>

namespace hf {

    class RX {

        public:

            RX(HardwareSerial * serial) : _serial(serial) { }

            static void begin(HardwareSerial *);

            void begin()
            {
                begin(_serial);
            }

            auto read() -> RxData;

        private:

            HardwareSerial * _serial;
    };
}
