/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under the
   terms of the GNU General Public License as published by the Free Software
   Foundation, either version 3 of the License, or (at your option) any later
   version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
   WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
   PARTICULAR PURPOSE. See the GNU General Public License for more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "datatypes.h"
#include "led.h"
#include "motor.h"

class Arming {

    public:

        typedef struct {

            bool acc_done_calibrating;
            bool angle_okay;
            bool switch_okay;
            bool gyro_done_calibrating;
            bool is_armed;
            Led  led;
            bool rx_failsafe_okay;
            bool throttle_is_down;

        } data_t;

    private:

        static bool readyToArm(data_t * data)
        {
            return 
                data->acc_done_calibrating &&
                data->angle_okay &&
                data->switch_okay &&
                data->gyro_done_calibrating &&
                data->rx_failsafe_okay &&
                data->throttle_is_down;
        }

        static bool rxAux1IsSet(float raw[])
        {
            return raw[4] > 1200;
        }

    public:

        static void check(
                data_t * data,
                void * motorDevice,
                uint32_t currentTimeUs,
                float raw[],
                bool imuIsLevel,
                bool calibrating)
        {
            static bool _doNotRepeat;

            if (rxAux1IsSet(raw)) {

                Arming::updateStatus(data, raw, imuIsLevel, calibrating);

                if (readyToArm(data)) {

                    if (data->is_armed) {
                        return;
                    }

                    if (!motorIsReady(currentTimeUs)) {
                        return;
                    }

                    data->is_armed = true;

                }

            } else {

                if (data->is_armed) {
                    Arming::disarm(data, motorDevice);
                    data->is_armed = false;
                }
            }

            if (!(data->is_armed || _doNotRepeat || !readyToArm(data))) {
                _doNotRepeat = true;
            }
        }

        static void disarm(data_t * data, void * motorDevice)
        {
            if (data->is_armed) {
                motorStop(motorDevice);
            }

            data->is_armed = false;
        }

        static bool isArmed(data_t * data)
        {
            return data->is_armed;
        }

        static void updateStatus(
                data_t * data,
                float raw[],
                bool imuIsLevel,
                bool calibrating) 
        {
            if (data->is_armed) {
                ledDevSet(true);
            } else {

                data->throttle_is_down = throttleIsDown(raw);

                data->angle_okay = imuIsLevel;

                data->gyro_done_calibrating = !calibrating;

                data->acc_done_calibrating = true;

                // If arming is disabled and the ARM switch is on
                if (!readyToArm(data) && rxAux1IsSet(raw)) {
                    data->switch_okay = false;
                } else if (!rxAux1IsSet(raw)) {
                    data->switch_okay = true;
                }

                if (!readyToArm(data)) {
                    ledWarningFlash();
                } else {
                    ledWarningDisable();
                }

                ledWarningUpdate();
            }
        }

        static void setRxFailsafe(data_t * data, bool okay)
        {
            data->rx_failsafe_okay= okay;
        }

        static bool throttleIsDown(float raw[])
        {
            return raw[THROTTLE] < 1050;
        }

}; // class Arming
