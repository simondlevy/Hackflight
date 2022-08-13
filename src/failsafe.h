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

        typedef struct m_state_s {
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
        } m_state_t;

        m_state_t m_state;

        void activate(void)
        {
            m_state.active = true;

            m_state.phase = LANDING;

            m_state.landingShouldBeFinishedAt =
                timeMillis() + 10 * MILLIS_PER_TENTH_SECOND;

            m_state.events++;
        }


        bool isReceivingRxData(void)
        {
            return (m_state.rxLinkState == RXLINK_UP);
        }

    public:

        Failsafe(void)
        {
            m_state.events = 0;
            m_state.monitoring = false;
        }

        bool isMonitoring(void)
        {
            return m_state.monitoring;
        }

        bool isActive(void)
        {
            return m_state.active;
        }

        void onValidDataFailed(Arming::data_t * arming)
        {
            (void)arming;
            Arming::setRxFailsafe(arming, false);
            m_state.validRxDataFailedAt = timeMillis();
            if ((m_state.validRxDataFailedAt - m_state.validRxDataReceivedAt) >
                    m_state.rxDataFailurePeriod) {
                m_state.rxLinkState = RXLINK_DOWN;
            }
        }

        void onValidDataReceived(Arming::data_t * arming)
        {
            m_state.validRxDataReceivedAt = timeMillis();
            if ((m_state.validRxDataReceivedAt - m_state.validRxDataFailedAt) >
                    m_state.rxDataRecoveryPeriod) {
                m_state.rxLinkState = RXLINK_UP;
                Arming::setRxFailsafe(arming, true);
            }
        }

        void reset(void)
        {
            m_state.rxDataFailurePeriod =
                PERIOD_RXDATA_FAILURE + 4 * MILLIS_PER_TENTH_SECOND;
            m_state.rxDataRecoveryPeriod =
                PERIOD_RXDATA_RECOVERY + 20 * MILLIS_PER_TENTH_SECOND;
            m_state.validRxDataReceivedAt = 0;
            m_state.validRxDataFailedAt = 0;
            m_state.throttleLowPeriod = 0;
            m_state.landingShouldBeFinishedAt = 0;
            m_state.receivingRxDataPeriod = 0;
            m_state.receivingRxDataPeriodPreset = 0;
            m_state.phase = IDLE;
            m_state.rxLinkState = RXLINK_DOWN;
        }        

        void startMonitoring(void)
        {
            m_state.monitoring = true;
        }

        void update(float * rcData, void * motorDevice, Arming::data_t * arming)
        {
            if (!isMonitoring()) {
                return;
            }

            bool receivingRxData = isReceivingRxData();

            if (0 == SWITCH_MODE_STAGE2) {
                receivingRxData = false; // force Stage2
            }

            bool reprocessState;

            do {
                reprocessState = false;

                switch (m_state.phase) {
                    case IDLE:
                        if (Arming::isArmed(arming)) {
                            // Track throttle command below minimum time
                            if (!Arming::throttleIsDown(rcData)) {
                                m_state.throttleLowPeriod =
                                    timeMillis() + 100 * MILLIS_PER_TENTH_SECOND;
                            }
                            if (0 == SWITCH_MODE_KILL) {
                                activate();
                                m_state.phase = LANDED;      
                                m_state.receivingRxDataPeriodPreset =
                                    PERIOD_OF_1_SECONDS();    
                                // require 1 seconds of valid rxData
                                reprocessState = true;
                            } else if (!receivingRxData) {
                                if (timeMillis() > m_state.throttleLowPeriod
                                   ) {
                                    activate();

                                    // skip auto-landing procedure
                                    m_state.phase = LANDED;      
                                    m_state.receivingRxDataPeriodPreset =
                                        PERIOD_OF_3_SECONDS(); 
                                    // require 3 seconds of valid rxData
                                } else {
                                    m_state.phase = RX_LOSS_DETECTED;
                                }
                                reprocessState = true;
                            }
                        } else {
                            m_state.throttleLowPeriod = 0;
                        }
                        break;

                    case RX_LOSS_DETECTED:
                        if (receivingRxData) {
                            m_state.phase = RX_LOSS_RECOVERED;
                        } else {
                            // Drop the craft
                            activate();

                            // skip auto-landing procedure
                            m_state.phase = LANDED;      
                            break;
                        }
                        reprocessState = true;
                        break;

                    case LANDING:
                        break;
                    case LANDED:
                        Arming::disarm(arming, motorDevice);
                        m_state.receivingRxDataPeriod = timeMillis() +
                            m_state.receivingRxDataPeriodPreset; // set required
                        m_state.phase = RX_LOSS_MONITORING;
                        reprocessState = true;
                        break;

                    case RX_LOSS_MONITORING:
                        if (receivingRxData) {
                            if (timeMillis() > m_state.receivingRxDataPeriod) {
                                if (!Arming::isArmed(arming)) {
                                    m_state.phase = RX_LOSS_RECOVERED;
                                    reprocessState = true;
                                }
                            }
                        } else {
                            m_state.receivingRxDataPeriod = timeMillis() +
                                m_state.receivingRxDataPeriodPreset; 
                        }
                        break;

                    case RX_LOSS_RECOVERED:
                        m_state.throttleLowPeriod = timeMillis() + 100 *
                            MILLIS_PER_TENTH_SECOND;
                        m_state.phase = IDLE;
                        m_state.active = false;
                        reprocessState = true;
                        break;

                    default:
                        break;
                }
            } while (reprocessState);        
        }
};
