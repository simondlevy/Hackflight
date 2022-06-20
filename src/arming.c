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
#include "led.h"
#include "motor.h"
#include "rx_throttle_status.h"

static arming_t status;

void armingDisarm(bool armed)
{
    if (armed) {
        motorStop();
    }
}

uint8_t armingGetDisableFlags(void)
{
    return ffs(status.disabledFlags);
}

bool armingIsDisabled(void)
{
    return status.disabledFlags != 0;
}

void armingSetDisabled(uint8_t flag)
{
    status.disabledFlags |= (1 << flag);
}

void armingTryArm(
        uint32_t currentTimeUs,
        uint8_t * tryingToArm,
        float raw[],
        bool imuIsLevel,
        bool calibrating,
        bool * armed)
{
    armingUpdateStatus(currentTimeUs, raw, imuIsLevel, calibrating, *armed);

    if (!armingIsDisabled()) {
        if (*armed) {
            return;
        }

        if (!motorCheckDshotReady(currentTimeUs, tryingToArm)) {
            return;
        }

        *armed = true;

        resetTryingToArm(tryingToArm);

    } else {
        resetTryingToArm(tryingToArm);
    }
}

void armingUpdateStatus(
        uint32_t currentTimeUs,
        float raw[],
        bool imuIsLevel,
        bool calibrating,
        bool armed)
{
    if (armed) {
        ledSet(true);
    } else {

        // Check if the power on arming grace time has elapsed
        if ((armingGetDisableFlags() & ARMING_DISABLED_BOOT_GRACE_TIME) &&
                (currentTimeUs >= 5000000)
                && (!motorIsProtocolDshot() || motorDshotStreamingCommandsAreEnabled())
           ) {
            // If so, unset the grace time arming disable flag
            armingUnsetDisabled(ARMING_DISABLED_BOOT_GRACE_TIME);
        }

        if (rxCalculateThrottleStatus(raw) != THROTTLE_LOW) {
            armingSetDisabled(ARMING_DISABLED_THROTTLE);
        } else {
            armingUnsetDisabled(ARMING_DISABLED_THROTTLE);
        }

        if (!imuIsLevel) {
            armingSetDisabled(ARMING_DISABLED_ANGLE);
        } else {
            armingUnsetDisabled(ARMING_DISABLED_ANGLE);
        }

        if (calibrating) {
            armingSetDisabled(ARMING_DISABLED_CALIBRATING);
        } else {
            armingUnsetDisabled(ARMING_DISABLED_CALIBRATING);
        }

        armingUnsetDisabled(ARMING_DISABLED_RPMFILTER);

        motorCheckDshotBitbangStatus();

        armingUnsetDisabled(ARMING_DISABLED_ACC_CALIBRATION);

        if (!motorIsProtocolEnabled()) {
            armingSetDisabled(ARMING_DISABLED_MOTOR_PROTOCOL);
        }

        // If arming is disabled and the ARM switch is on
        if (armingIsDisabled() && isAux1Set(raw)) {
            armingSetDisabled(ARMING_DISABLED_ARM_SWITCH);
        } else if (!isAux1Set(raw)) {
            armingUnsetDisabled(ARMING_DISABLED_ARM_SWITCH);
        }

        if (armingIsDisabled()) {
            ledWarningFlash();
        } else {
            ledWarningDisable();
        }

        ledWarningUpdate();
    }
}

void armingUnsetDisabled(uint8_t flag)
{
    status.disabledFlags &= ~(1 << flag);
}

