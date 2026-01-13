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

#include <datatypes.h>
#include <firmware/timer.hpp>

class Led {

    public:

        void init()
        {
            device_init();
        }

        void run(
                const uint32_t msec_curr,
                const bool imuIsCalibrated,
                const mode_e status)
        {
            if (!imuIsCalibrated) {
                blink(msec_curr, IMU_CALIBRATING_FREQ);
            }

            else if (status == MODE_ARMED ||
                    status == MODE_HOVERING || 
                    status == MODE_LANDING) { 
                device_set(true);
            }
            else {
                blink(msec_curr, HEARTBEAT_FREQ);
            }
        }

    private:

        static constexpr float HEARTBEAT_FREQ = 1;
        static constexpr float IMU_CALIBRATING_FREQ = 3;
        static constexpr uint32_t PULSE_DURATION_MSEC = 50;

        void blink(const uint32_t msec_curr, const float freq)
        {
            static bool _pulsing;
            static uint32_t _pulse_start;
            static Timer _timer;

            if (_timer.ready(freq)) {
                device_set(true);
                _pulsing = true;
                _pulse_start = msec_curr;
            }

            else if (_pulsing) {
                if (millis() - _pulse_start > PULSE_DURATION_MSEC) {
                    device_set(false);
                    _pulsing = false;
                }
            }
        }

        void device_init();

        void device_set(const bool on);
};
