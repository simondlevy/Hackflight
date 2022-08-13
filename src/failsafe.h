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
            IDLE = 0,
            RX_LOSS_DETECTED,
            LANDING,
            LANDED,
            RX_LOSS_MONITORING,
            RX_LOSS_RECOVERED
        } failsafePhase_e;

        typedef enum {
            RXLINK_DOWN = 0,
            RXLINK_UP
        } failsafeRxLinkState_e;

        typedef enum {
            PROCEDURE_AUTO_LANDING = 0,
            PROCEDURE_DROP_IT,
            PROCEDURE_COUNT   // must be last
        } failsafeProcedure_e;

        typedef enum {
            SWITCH_MODE_STAGE1 = 0,
            SWITCH_MODE_KILL,
            SWITCH_MODE_STAGE2
        } failsafeSwitchMode_e;

        typedef struct {
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
        } data_t;

        static void activate(data_t * data)
        {
            data->active = true;

            data->phase = LANDING;

            data->landingShouldBeFinishedAt =
                timeMillis() + 10 * MILLIS_PER_TENTH_SECOND;

            data->events++;
        }


        static bool isReceivingRxData(data_t * data)
        {
            return (data->rxLinkState == RXLINK_UP);
        }

    public:

        static void init(data_t * data)
        {
            data->events = 0;
            data->monitoring = false;
        }

        static bool isMonitoring(data_t * data)
        {
            return data->monitoring;
        }

        static bool isActive(data_t * data)
        {
            return data->active;
        }

        static void onValidDataFailed(data_t * data, Arming::data_t * arming)
        {
            (void)arming;
            Arming::setRxFailsafe(arming, false);
            data->validRxDataFailedAt = timeMillis();
            if ((data->validRxDataFailedAt - data->validRxDataReceivedAt) >
                    data->rxDataFailurePeriod) {
                data->rxLinkState = RXLINK_DOWN;
            }
        }

        static void onValidDataReceived(data_t * data, Arming::data_t * arming)
        {
            data->validRxDataReceivedAt = timeMillis();
            if ((data->validRxDataReceivedAt - data->validRxDataFailedAt) >
                    data->rxDataRecoveryPeriod) {
                data->rxLinkState = RXLINK_UP;
                Arming::setRxFailsafe(arming, true);
            }
        }

        static void reset(data_t * data)
        {
            data->rxDataFailurePeriod =
                PERIOD_RXDATA_FAILURE + 4 * MILLIS_PER_TENTH_SECOND;
            data->rxDataRecoveryPeriod =
                PERIOD_RXDATA_RECOVERY + 20 * MILLIS_PER_TENTH_SECOND;
            data->validRxDataReceivedAt = 0;
            data->validRxDataFailedAt = 0;
            data->throttleLowPeriod = 0;
            data->landingShouldBeFinishedAt = 0;
            data->receivingRxDataPeriod = 0;
            data->receivingRxDataPeriodPreset = 0;
            data->phase = IDLE;
            data->rxLinkState = RXLINK_DOWN;
        }        

        static void startMonitoring(data_t * data)
        {
            data->monitoring = true;
        }

        static void update(data_t * data, float * rcData, void * motorDevice, Arming::data_t * arming)
        {
            if (!isMonitoring(data)) {
                return;
            }

            bool receivingRxData = isReceivingRxData(data);

            if (0 == SWITCH_MODE_STAGE2) {
                receivingRxData = false; // force Stage2
            }

            bool reprocessState;

            do {
                reprocessState = false;

                switch (data->phase) {
                    case IDLE:
                        if (Arming::isArmed(arming)) {
                            // Track throttle command below minimum time
                            if (!Arming::throttleIsDown(rcData)) {
                                data->throttleLowPeriod =
                                    timeMillis() + 100 * MILLIS_PER_TENTH_SECOND;
                            }
                            if (0 == SWITCH_MODE_KILL) {
                                activate(data);
                                data->phase = LANDED;      
                                data->receivingRxDataPeriodPreset =
                                    PERIOD_OF_1_SECONDS();    
                                // require 1 seconds of valid rxData
                                reprocessState = true;
                            } else if (!receivingRxData) {
                                if (timeMillis() > data->throttleLowPeriod
                                   ) {
                                    activate(data);

                                    // skip auto-landing procedure
                                    data->phase = LANDED;      
                                    data->receivingRxDataPeriodPreset =
                                        PERIOD_OF_3_SECONDS(); 
                                    // require 3 seconds of valid rxData
                                } else {
                                    data->phase = RX_LOSS_DETECTED;
                                }
                                reprocessState = true;
                            }
                        } else {
                            data->throttleLowPeriod = 0;
                        }
                        break;

                    case RX_LOSS_DETECTED:
                        if (receivingRxData) {
                            data->phase = RX_LOSS_RECOVERED;
                        } else {
                            // Drop the craft
                            activate(data);

                            // skip auto-landing procedure
                            data->phase = LANDED;      
                            break;
                        }
                        reprocessState = true;
                        break;

                    case LANDING:
                        break;
                    case LANDED:
                        Arming::disarm(arming, motorDevice);
                        data->receivingRxDataPeriod = timeMillis() +
                            data->receivingRxDataPeriodPreset; // set required
                        data->phase = RX_LOSS_MONITORING;
                        reprocessState = true;
                        break;

                    case RX_LOSS_MONITORING:
                        if (receivingRxData) {
                            if (timeMillis() > data->receivingRxDataPeriod) {
                                if (!Arming::isArmed(arming)) {
                                    data->phase = RX_LOSS_RECOVERED;
                                    reprocessState = true;
                                }
                            }
                        } else {
                            data->receivingRxDataPeriod = timeMillis() +
                                data->receivingRxDataPeriodPreset; 
                        }
                        break;

                    case RX_LOSS_RECOVERED:
                        data->throttleLowPeriod = timeMillis() + 100 *
                            MILLIS_PER_TENTH_SECOND;
                        data->phase = IDLE;
                        data->active = false;
                        reprocessState = true;
                        break;

                    default:
                        break;
                }
            } while (reprocessState);        
        }
};
