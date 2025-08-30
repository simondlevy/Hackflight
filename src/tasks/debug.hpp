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

#include <stdarg.h>
#include <string.h>
#include <task.hpp>
#include <usb_api.h>

class DebugTask {

    public:

        void begin()
        {
            _task.init(runDebugCommsTask, "debug", this, 2);
		}

        static void setMessage(DebugTask * debugTask, const char * format, ...)
        {
            if (debugTask) {

                va_list args = {};

                char buffer[256] = {};

                va_start(args, format);

                const auto vsErr = vsprintf(buffer, format, args);

                if (vsErr >= 0) { 
                    strcpy(debugTask->_msg, buffer);
                }

                va_end(args);
            }
        }

    private:

        static constexpr float REPORT_FREQ = 10;

        FreeRtosTask _task;

        char _msg[100];

        static void runDebugCommsTask(void * obj)
        {
            ((DebugTask *)obj)->run();
        }

        void run(void)
        {
            while (true) {

                if (*_msg) {
                    usbWrite(_msg);
                }

                vTaskDelay(M2T(1000/REPORT_FREQ));
            }
        }
};

