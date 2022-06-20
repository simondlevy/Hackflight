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
    uint32_t disabledFlags;
} arming_t;

enum {
    ARMING_DELAYED_DISARMED = 0,
    ARMING_DELAYED_NORMAL = 1,
    ARMING_DELAYED_LAUNCH_CONTROL = 3,
};

// Arming disable flags are listed in the order of importance
enum {
    ARMING_DISABLED_NO_GYRO         ,
    ARMING_DISABLED_FAILSAFE        ,
    ARMING_DISABLED_RX_FAILSAFE     ,
    ARMING_DISABLED_BAD_RX_RECOVERY ,
    ARMING_DISABLED_BOXFAILSAFE     ,
    ARMING_DISABLED_THROTTLE        ,
    ARMING_DISABLED_ANGLE           ,
    ARMING_DISABLED_BOOT_GRACE_TIME ,
    ARMING_DISABLED_NOPREARM        ,
    ARMING_DISABLED_CALIBRATING     ,
    ARMING_DISABLED_CLI             ,
    ARMING_DISABLED_CMS_MENU        ,
    ARMING_DISABLED_BST             ,
    ARMING_DISABLED_MSP             ,
    ARMING_DISABLED_PARALYZE        ,
    ARMING_DISABLED_RESC            ,
    ARMING_DISABLED_RPMFILTER       ,
    ARMING_DISABLED_REBOOT_REQUIRED ,
    ARMING_DISABLED_DSHOT_BITBANG   ,
    ARMING_DISABLED_ACC_CALIBRATION ,
    ARMING_DISABLED_MOTOR_PROTOCOL  ,
    ARMING_DISABLED_ARM_SWITCH
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

    void  armingSetDisabled(uint8_t flag);

    void  armingUnsetDisabled(uint8_t flag);

    void  armingUpdateStatus(
            uint32_t currentTimeUs,
            float raw[],
            bool imuIsLevel,
            bool calibrating,
            bool armed);

#if defined(__cplusplus)
}
#endif
