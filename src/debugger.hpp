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

#include <Arduino.h>

#include <bootloader.hpp>
#include <timer.hpp>

class Debugger {

    public:

        static void setMessage(Debugger * debugger, const char * format, ...)
        {
            if (debugger) {

                va_list args = {};

                char buffer[256] = {};

                va_start(args, format);

                const auto vsErr = vsprintf(buffer, format, args);

                if (vsErr >= 0) { 
                    strcpy(debugger->_msg, buffer);
                }

                va_end(args);
            }
        }

        void step()
        {
            if (_timer.ready(REPORT_FREQ)) {

                if (*_msg != 0) {
                    Serial.println(_msg);
                }

                if (Serial.available() && Serial.read() == 'R') {
                    Bootloader::jump();
                }
            }
        }

    private:

        static constexpr float REPORT_FREQ = 10;

        Timer _timer;

        char _msg[100];
};

