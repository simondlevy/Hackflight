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

void failsafeInit(void);
void failsafeReset(void);
void failsafeStartMonitoring(void);
void failsafeUpdateState(float * rcData, void * motorDevice, Arming::data_t * arming);
bool failsafeIsMonitoring(void);
bool failsafeIsActive(void);
void failsafeOnValidDataReceived(Arming::data_t * arming);
void failsafeOnValidDataFailed(Arming::data_t * arming);

class Failsafe {

    private:

        static const uint32_t MILLIS_PER_TENTH_SECOND    =   100;
        static const uint32_t MILLIS_PER_SECOND          =  1000;

        static uint32_t PERIOD_OF_1_SECONDS() { return 1 * MILLIS_PER_SECOND; }
        static uint32_t PERIOD_OF_3_SECONDS() { return 3 * MILLIS_PER_SECOND; }

        static const uint32_t PERIOD_RXDATA_FAILURE      =   200;    // millis
        static const uint32_t PERIOD_RXDATA_RECOVERY     =   200;    // millis

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
            uint32_t throttleLowPeriod;             
            uint32_t landingShouldBeFinishedAt;
            uint32_t receivingRxDataPeriod;        
            uint32_t receivingRxDataPeriodPreset; 
            failsafePhase_e phase;
            failsafeRxLinkState_e rxLinkState;
        } failsafeState_t;

        static failsafeState_t state;

    public:

        void init(void)
        {
            state.events = 0;
            state.monitoring = false;
        }

        bool isMonitoring(void)
        {
            return state.monitoring;
        }

        bool isActive(void)
        {
            return state.active;
        }

        void onValidDataFailed(Arming::data_t * arming)
        {
            (void)arming;
            Arming::setRxFailsafe(arming, false);
            state.validRxDataFailedAt = timeMillis();
            if ((state.validRxDataFailedAt - state.validRxDataReceivedAt) >
                    state.rxDataFailurePeriod) {
                state.rxLinkState = FAILSAFE_RXLINK_DOWN;
            }
        }

        void onValidDataReceived(Arming::data_t * arming)
        {
            (void)arming;
        }

        void reset(void)
        {
            state.rxDataFailurePeriod =
                PERIOD_RXDATA_FAILURE + 4 * MILLIS_PER_TENTH_SECOND;
            state.rxDataRecoveryPeriod =
                PERIOD_RXDATA_RECOVERY + 20 * MILLIS_PER_TENTH_SECOND;
            state.validRxDataReceivedAt = 0;
            state.validRxDataFailedAt = 0;
            state.throttleLowPeriod = 0;
            state.landingShouldBeFinishedAt = 0;
            state.receivingRxDataPeriod = 0;
            state.receivingRxDataPeriodPreset = 0;
            state.phase = FAILSAFE_IDLE;
            state.rxLinkState = FAILSAFE_RXLINK_DOWN;
        }        

        void startMonitoring(void)
        {
        }

        void updateState(
                float * rcData, void * motorDevice, Arming::data_t * arming)
        {
            (void)rcData;
            (void)motorDevice;
            (void)arming;
        }
};
