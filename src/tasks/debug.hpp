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

#if defined(CORE_TEENSY)

#include <Arduino.h>
#include <string.h>
#include <task.hpp>

class DebugTask {

    public:

        void begin()
        {
            if (_task.didInit()){
                return;
            }

            _task.init(runDebugCommsTask, "debug", this, 2);
		}

		void setMessage(const char * format, ...)
		{
			va_list args = {};

			char buffer[256] = {};

			va_start(args, format);

			const auto vsErr = vsprintf(buffer, format, args);

			if (vsErr >= 0) { 
				strcpy(_msg, buffer);
			}

			va_end(args);
		}

	private:

		static constexpr float REPORT_FREQ = 100;

		FreeRtosTask _task;

		char _msg[100];

		static void runDebugCommsTask(void * obj)
		{
			((DebugTask *)obj)->run();
		}

		void run(void)
		{
			systemWaitStart();

			while (true) {

				if (*_msg) {
					Serial.println(_msg);
				}

				vTaskDelay(M2T(1000/REPORT_FREQ));
			}
		}
};

#endif
