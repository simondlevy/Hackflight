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

typedef struct {

    bool acc_done_calibrating;
    bool angle_okay;
    bool arming_switch_okay;
    bool gyro_done_calibrating;
    bool boot_grace_time_done;
    bool dshot_bitbang_okay;
    bool rx_failsafe_okay;
    bool throttle_is_down;

} arming_t;

enum {
    ARMING_DELAYED_DISARMED = 0,
    ARMING_DELAYED_NORMAL = 1,
    ARMING_DELAYED_LAUNCH_CONTROL = 3,
};

#if defined(__cplusplus)
extern "C" {
#endif

    void armingCheck(
            uint32_t currentTimeUs,
            bool signalReceived,
            float raw[],
            bool imuIsLevel,
            bool calibrating,
            bool * armed);

    void  armingDisarm(bool armed);

    void  armingUpdateStatus(
            uint32_t currentTimeUs,
            float raw[],
            bool imuIsLevel,
            bool calibrating,
            bool armed);

    void armingSetDshotBitbang(bool enabled);
    void armingSetRxFailsafe(bool enabled);

#if defined(__cplusplus)
}
#endif
