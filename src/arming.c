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

#include <strings.h>

#include "arming.h"
#include "debug.h"
#include "failsafe.h"
#include "led.h"
#include "motor.h"
#include "rx_status.h"

static arming_t _status;

static bool readyToArm(void)
{
    return 
        _status.acc_done_calibrating &&
        _status.angle_okay &&
        _status.arming_switch_okay &&
        _status.gyro_done_calibrating &&
        _status.dshot_bitbang_okay &&
        _status.rx_failsafe_okay &&
        _status.throttle_is_down;
}

void armingCheck(
        uint32_t currentTimeUs,
        float raw[],
        bool imuIsLevel,
        bool calibrating,
        bool * armed)
{
    static bool _doNotRepeat;

    if (rxAux1IsSet(raw)) {

        armingUpdateStatus(raw, imuIsLevel, calibrating, *armed);

        if (readyToArm()) {

            if (*armed) {
                return;
            }

            if (!motorIsReady(currentTimeUs)) {
                return;
            }

            *armed = true;

        }

    } else {

        if (*armed) {
            armingDisarm(*armed);
            *armed = false;
        }
    }

    if (!(*armed || _doNotRepeat || !readyToArm())) {
        _doNotRepeat = true;
    }
}

void armingDisarm(bool armed)
{
    if (armed) {
        motorStop();
    }
}


void armingUpdateStatus( float raw[], bool imuIsLevel, bool calibrating, bool armed)
{
    if (armed) {
        ledSet(true);
    } else {

        _status.throttle_is_down = rxThrottleIsDown(raw);

        _status.angle_okay = imuIsLevel;

        _status.gyro_done_calibrating = !calibrating;

        motorCheckDshotBitbangStatus();

        _status.acc_done_calibrating = true;

        // If arming is disabled and the ARM switch is on
        if (!readyToArm() && rxAux1IsSet(raw)) {
            _status.arming_switch_okay = false;
        } else if (!rxAux1IsSet(raw)) {
            _status.arming_switch_okay = true;
        }

        if (!readyToArm()) {
            ledWarningFlash();
        } else {
            ledWarningDisable();
        }

        ledWarningUpdate();
    }
}

void armingSetDshotBitbang(bool okay)
{
    _status.dshot_bitbang_okay = okay;
}

void armingSetRxFailsafe(bool okay)
{
    _status.rx_failsafe_okay= okay;
}

