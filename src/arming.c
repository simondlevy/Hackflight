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
#include "rx_throttle_status.h"

static arming_t _status;

static void resetTryingToArm(uint8_t * tryingToArm)
{
    *tryingToArm = ARMING_DELAYED_DISARMED;
}

static uint8_t getDisableFlags(void)
{
    return ffs(_status.disabledFlags);
}

static bool isDisabled(void)
{
    return _status.disabledFlags != 0;
}


void armingCheck(
        uint32_t currentTimeUs,
        bool signalReceived,
        float raw[],
        bool imuIsLevel,
        bool calibrating,
        bool * armed)
{
    static uint8_t _disarmTicks;
    static bool _doNotRepeat;
    static uint8_t _tryingToArm;

    if (isAux1Set(raw)) {
        _disarmTicks = 0;

        armingUpdateStatus(currentTimeUs, raw, imuIsLevel, calibrating, *armed);

        if (!isDisabled()) {
            if (*armed) {
                return;
            }

            if (!motorCheckDshotReady(currentTimeUs, &_tryingToArm)) {
                return;
            }

            *armed = true;

            resetTryingToArm(&_tryingToArm);

        } else {
            resetTryingToArm(&_tryingToArm);
        }

    } else {
        resetTryingToArm(&_tryingToArm);
        if (*armed && signalReceived && !failsafeIsActive()  ) {
            _disarmTicks++;
            if (_disarmTicks > 3) {
                armingDisarm(*armed);
                *armed = false;
            }
        }
    }

    if (!(*armed || _doNotRepeat || isDisabled())) {
        _doNotRepeat = true;
    }
}

void armingDisarm(bool armed)
{
    if (armed) {
        motorStop();
    }
}

void armingSetDisabled(uint8_t flag)
{
    _status.disabledFlags |= (1 << flag);
}

void armingSetEnabled(uint8_t flag)
{
    _status.disabledFlags &= ~(1 << flag);
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

        //debugPrintf("flags: %02X  mask %02X\n", getDisableFlags(), ARMING_DISABLED_BOOT_GRACE_TIME);

        // Check if the power on arming grace time has elapsed
        if ((getDisableFlags() & ARMING_DISABLED_BOOT_GRACE_TIME) &&
                (currentTimeUs >= 5000000) &&
                (!motorIsProtocolDshot() || motorDshotStreamingCommandsAreEnabled())
           ) {
            // If so, unset the grace time arming disable flag
            armingSetEnabled(ARMING_DISABLED_BOOT_GRACE_TIME);
        }

        if (rxCalculateThrottleStatus(raw) != THROTTLE_LOW) {
            armingSetDisabled(ARMING_DISABLED_THROTTLE);
        } else {
            armingSetEnabled(ARMING_DISABLED_THROTTLE);
        }

        if (!imuIsLevel) {
            armingSetDisabled(ARMING_DISABLED_ANGLE);
        } else {
            armingSetEnabled(ARMING_DISABLED_ANGLE);
        }

        if (calibrating) {
            armingSetDisabled(ARMING_DISABLED_CALIBRATING);
        } else {
            armingSetEnabled(ARMING_DISABLED_CALIBRATING);
        }

        armingSetEnabled(ARMING_DISABLED_RPMFILTER);

        motorCheckDshotBitbangStatus();

        armingSetEnabled(ARMING_DISABLED_ACC_CALIBRATION);

        if (!motorIsProtocolEnabled()) {
            armingSetDisabled(ARMING_DISABLED_MOTOR_PROTOCOL);
        }

        // If arming is disabled and the ARM switch is on
        if (isDisabled() && isAux1Set(raw)) {
            armingSetDisabled(ARMING_DISABLED_ARM_SWITCH);
        } else if (!isAux1Set(raw)) {
            armingSetEnabled(ARMING_DISABLED_ARM_SWITCH);
        }

        if (isDisabled()) {
            ledWarningFlash();
        } else {
            ledWarningDisable();
        }

        ledWarningUpdate();
    }
}
