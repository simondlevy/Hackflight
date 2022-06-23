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

static bool readyToArm(arming_t * arming)
{
    return 
        arming->acc_done_calibrating &&
        arming->angle_okay &&
        arming->arming_switch_okay &&
        arming->gyro_done_calibrating &&
        arming->rx_failsafe_okay &&
        arming->throttle_is_down &&
        arming->dshot_bitbang_okay;
}

void armingCheck(
        arming_t * arming,
        void * motorDevice,
        uint32_t currentTimeUs,
        float raw[],
        bool imuIsLevel,
        bool calibrating)
{
    static bool _doNotRepeat;

    if (rxAux1IsSet(raw)) {

        armingUpdateStatus(arming, raw, imuIsLevel, calibrating);

        if (readyToArm(arming)) {

            if (arming->is_armed) {
                return;
            }

            if (!motorIsReady(currentTimeUs)) {
                return;
            }

            arming->is_armed = true;

        }

    } else {

        if (arming->is_armed) {
            armingDisarm(arming, motorDevice);
            arming->is_armed = false;
        }
    }

    if (!(arming->is_armed || _doNotRepeat || !readyToArm(arming))) {
        _doNotRepeat = true;
    }
}

void armingDisarm(arming_t * arming, void * motorDevice)
{
    if (arming->is_armed) {
        motorStop(motorDevice);
    }

    arming->is_armed = false;
}

bool armingIsArmed(arming_t * arming)
{
    return arming->is_armed;
}

void armingUpdateStatus(
        arming_t * arming,
        float raw[],
        bool imuIsLevel,
        bool calibrating)
{
    if (arming->is_armed) {
        ledSet(true);
    } else {

        arming->throttle_is_down = rxThrottleIsDown(raw);

        arming->angle_okay = imuIsLevel;

        arming->gyro_done_calibrating = !calibrating;

        motorCheckDshotBitbangStatus(arming);

        arming->acc_done_calibrating = true;

        // If arming is disabled and the ARM switch is on
        if (!readyToArm(arming) && rxAux1IsSet(raw)) {
            arming->arming_switch_okay = false;
        } else if (!rxAux1IsSet(raw)) {
            arming->arming_switch_okay = true;
        }

        if (!readyToArm(arming)) {
            ledWarningFlash();
        } else {
            ledWarningDisable();
        }

        ledWarningUpdate();
    }
}

void armingSetDshotBitbangOkay(arming_t * arming, bool okay)
{
    arming->dshot_bitbang_okay = okay;
}

void armingSetRxFailsafe(arming_t * arming, bool okay)
{
    arming->rx_failsafe_okay= okay;
}
