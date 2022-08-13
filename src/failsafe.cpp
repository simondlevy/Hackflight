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

#include "failsafe.h"

static const uint32_t MILLIS_PER_TENTH_SECOND    =   100;
static const uint32_t MILLIS_PER_SECOND          =  1000;

static uint32_t PERIOD_OF_1_SECONDS() { return 1 * MILLIS_PER_SECOND; }
static uint32_t PERIOD_OF_3_SECONDS() { return 3 * MILLIS_PER_SECOND; }

static const uint32_t PERIOD_RXDATA_FAILURE      =   200;    // millis
static const uint32_t PERIOD_RXDATA_RECOVERY     =   200;    // millis

/*
 * Usage:
 *
 * failsafeInit() and failsafeReset() must be called before the other methods
 * are used.
 *
 * failsafeInit() and failsafeReset() can be called in any order.
 * failsafeInit() should only be called once.
 *
 * enable() should be called after system initialisation.
 */

static failsafeState_t failsafeState;

const char * const failsafeProcedureNames[FAILSAFE_PROCEDURE_COUNT] = {
    "AUTO-LAND",
    "DROP",
};

/*
 * Should called when the failsafe config needs to be changed - e.g. a
 * different profile has been selected.
 */
void failsafeReset(void)
{
    failsafeState.rxDataFailurePeriod =
        PERIOD_RXDATA_FAILURE + 4 * MILLIS_PER_TENTH_SECOND;
    failsafeState.rxDataRecoveryPeriod =
        PERIOD_RXDATA_RECOVERY + 20 * MILLIS_PER_TENTH_SECOND;
    failsafeState.validRxDataReceivedAt = 0;
    failsafeState.validRxDataFailedAt = 0;
    failsafeState.throttleLowPeriod = 0;
    failsafeState.landingShouldBeFinishedAt = 0;
    failsafeState.receivingRxDataPeriod = 0;
    failsafeState.receivingRxDataPeriodPreset = 0;
    failsafeState.phase = FAILSAFE_IDLE;
    failsafeState.rxLinkState = FAILSAFE_RXLINK_DOWN;
}

void failsafeInit(void)
{
    failsafeState.events = 0;
    failsafeState.monitoring = false;

    return;
}

bool failsafeIsMonitoring(void)
{
    return failsafeState.monitoring;
}

bool failsafeIsActive(void)
{
    return failsafeState.active;
}

void failsafeStartMonitoring(void)
{
    failsafeState.monitoring = true;
}

static void failsafeActivate(void)
{
    failsafeState.active = true;

    failsafeState.phase = FAILSAFE_LANDING;

    failsafeState.landingShouldBeFinishedAt =
        timeMillis() + 10 * MILLIS_PER_TENTH_SECOND;

    failsafeState.events++;
}

static bool failsafeIsReceivingRxData(void)
{
    return (failsafeState.rxLinkState == FAILSAFE_RXLINK_UP);
}

void failsafeOnValidDataReceived(Arming::data_t * arming)
{
    failsafeState.validRxDataReceivedAt = timeMillis();
    if ((failsafeState.validRxDataReceivedAt -
                failsafeState.validRxDataFailedAt) >
            failsafeState.rxDataRecoveryPeriod) { failsafeState.rxLinkState =
        FAILSAFE_RXLINK_UP;
        Arming::setRxFailsafe(arming, true);
    }
}

void failsafeOnValidDataFailed(Arming::data_t * arming)
{
    Arming::setRxFailsafe(arming, false);
    failsafeState.validRxDataFailedAt = timeMillis();
    if ((failsafeState.validRxDataFailedAt -
                failsafeState.validRxDataReceivedAt) >
            failsafeState.rxDataFailurePeriod) { failsafeState.rxLinkState =
        FAILSAFE_RXLINK_DOWN;
    }
}

void failsafeUpdateState(float * rcData, void * motorDevice, Arming::data_t * arming)
{
    if (!failsafeIsMonitoring()) {
        return;
    }

    bool receivingRxData = failsafeIsReceivingRxData();

    if (0 == FAILSAFE_SWITCH_MODE_STAGE2) {
        receivingRxData = false; // force Stage2
    }

    bool reprocessState;

    do {
        reprocessState = false;

        switch (failsafeState.phase) {
            case FAILSAFE_IDLE:
                if (Arming::isArmed(arming)) {
                    // Track throttle command below minimum time
                    if (!Arming::throttleIsDown(rcData)) {
                        failsafeState.throttleLowPeriod =
                            timeMillis() + 100 * MILLIS_PER_TENTH_SECOND;
                    }
                    // Kill switch logic (must be independent of
                    // receivingRxData to skip PERIOD_RXDATA_FAILURE delay
                    // before disarming)
                    if (0 == FAILSAFE_SWITCH_MODE_KILL) {
                        // KillswitchEvent: failsafe switch is configured as
                        // KILL switch and is switched ON
                        failsafeActivate();

                        // skip auto-landing procedure
                        failsafeState.phase = FAILSAFE_LANDED;      
                        failsafeState.receivingRxDataPeriodPreset =
                            PERIOD_OF_1_SECONDS();    
                        // require 1 seconds of valid rxData
                        reprocessState = true;
                    } else if (!receivingRxData) {
                        if (timeMillis() > failsafeState.throttleLowPeriod
                           ) {
                            // JustDisarm: throttle was LOW for at least
                            // 'failsafe_throttle_low_delay' seconds
                            failsafeActivate();

                            // skip auto-landing procedure
                            failsafeState.phase = FAILSAFE_LANDED;      
                            failsafeState.receivingRxDataPeriodPreset =
                                PERIOD_OF_3_SECONDS(); 
                            // require 3 seconds of valid rxData
                        } else {
                            failsafeState.phase = FAILSAFE_RX_LOSS_DETECTED;
                        }
                        reprocessState = true;
                    }
                } else {
                    // Throttle low period expired (= low long enough for JustDisarm)
                    failsafeState.throttleLowPeriod = 0;
                }
                break;

            case FAILSAFE_RX_LOSS_DETECTED:
                if (receivingRxData) {
                    failsafeState.phase = FAILSAFE_RX_LOSS_RECOVERED;
                } else {
                    // Drop the craft
                    failsafeActivate();

                    // skip auto-landing procedure
                    failsafeState.phase = FAILSAFE_LANDED;      
                    break;
                }
                reprocessState = true;
                break;

            case FAILSAFE_LANDING:
                break;
            case FAILSAFE_LANDED:
                Arming::disarm(arming, motorDevice);
                failsafeState.receivingRxDataPeriod = timeMillis() +
                    failsafeState.receivingRxDataPeriodPreset; // set required
                failsafeState.phase = FAILSAFE_RX_LOSS_MONITORING;
                reprocessState = true;
                break;

            case FAILSAFE_RX_LOSS_MONITORING:
                // Monitoring the rx link to allow rearming when it has become
                // good for > `receivingRxDataPeriodPreset` time.
                if (receivingRxData) {
                    if (timeMillis() > failsafeState.receivingRxDataPeriod) {
                        // rx link is good now, when arming via ARM switch, it
                        // must be OFF first
                        if (!Arming::isArmed(arming)) {
                            failsafeState.phase = FAILSAFE_RX_LOSS_RECOVERED;
                            reprocessState = true;
                        }
                    }
                } else {
                    failsafeState.receivingRxDataPeriod = timeMillis() +
                        failsafeState.receivingRxDataPeriodPreset; 
                }
                break;

            case FAILSAFE_RX_LOSS_RECOVERED:
                // Entering IDLE with the requirement that throttle first must
                // be at min_check for failsafe_throttle_low_delay period.
                // This is to prevent that JustDisarm is activated on the next
                // iteration.  Because that would have the effect of shutting
                // down failsafe handling on intermittent connections.
                failsafeState.throttleLowPeriod = timeMillis() + 100 *
                    MILLIS_PER_TENTH_SECOND;
                failsafeState.phase = FAILSAFE_IDLE;
                failsafeState.active = false;
                reprocessState = true;
                break;

            default:
                break;
        }
    } while (reprocessState);
}
