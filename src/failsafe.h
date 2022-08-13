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

#include <stdbool.h>
#include <stdint.h>

#include "arming.h"

typedef enum {
    FAILSAFE_IDLE = 0,
    FAILSAFE_RX_LOSS_DETECTED,
    FAILSAFE_LANDING,
    FAILSAFE_LANDED,
    FAILSAFE_RX_LOSS_MONITORING,
    FAILSAFE_RX_LOSS_RECOVERED
} failsafePhase_e;

typedef enum {
    FAILSAFE_RXLINK_DOWN = 0,
    FAILSAFE_RXLINK_UP
} failsafeRxLinkState_e;

typedef enum {
    FAILSAFE_PROCEDURE_AUTO_LANDING = 0,
    FAILSAFE_PROCEDURE_DROP_IT,
    FAILSAFE_PROCEDURE_COUNT   // must be last
} failsafeProcedure_e;

typedef enum {
    FAILSAFE_SWITCH_MODE_STAGE1 = 0,
    FAILSAFE_SWITCH_MODE_KILL,
    FAILSAFE_SWITCH_MODE_STAGE2
} failsafeSwitchMode_e;

typedef struct failsafeState_s {
    int16_t events;
    bool monitoring;
    bool active;
    uint32_t rxDataFailurePeriod;
    uint32_t rxDataRecoveryPeriod;
    uint32_t validRxDataReceivedAt;
    uint32_t validRxDataFailedAt;
    uint32_t throttleLowPeriod;             // throttle stick must have been below 'min_check' for this period
    uint32_t landingShouldBeFinishedAt;
    uint32_t receivingRxDataPeriod;         // period for the required period of valid rxData
    uint32_t receivingRxDataPeriodPreset;   // preset for the required period of valid rxData
    failsafePhase_e phase;
    failsafeRxLinkState_e rxLinkState;
} failsafeState_t;

void failsafeInit(void);
void failsafeReset(void);
void failsafeStartMonitoring(void);
void failsafeUpdateState(float * rcData, void * motorDevice, Arming::data_t * arming);
bool failsafeIsMonitoring(void);
bool failsafeIsActive(void);
bool failsafeIsReceivingRxData(void);
void failsafeOnValidDataReceived(Arming::data_t * arming);
void failsafeOnValidDataFailed(Arming::data_t * arming);

class Failsafe {


};
