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
#include "sticks.h"

class Failsafe {

    friend class AnglePidController;
    friend class Hackflight;
    friend class Receiver;
    friend class Task;

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

    bool                   m_active;
    int16_t                m_events;
    uint32_t               m_landingShouldBeFinishedAt;
    bool                   m_monitoring;
    failsafePhase_e        m_phase;
    uint32_t               m_receivingRxDataPeriod;        
    uint32_t               m_receivingRxDataPeriodPreset; 
    uint32_t               m_rxDataFailurePeriod;
    failsafeRxLinkState_e  m_rxLinkState;
    uint32_t               m_validRxDataReceivedAt;
    uint32_t               m_rxDataRecoveryPeriod;
    uint32_t               m_throttleLowPeriod;             
    uint32_t               m_validRxDataFailedAt;

    void activate(void)
    {
        m_active = true;

        m_phase = LANDING;

        m_landingShouldBeFinishedAt =
            timeMillis() + 10 * MILLIS_PER_TENTH_SECOND;

        m_events++;
    }


    bool isReceivingRxData(void)
    {
        return (m_rxLinkState == RXLINK_UP);
    }

    Failsafe()
    {
        m_events = 0;
        m_monitoring = false;

        reset();
    }

    bool isMonitoring(void)
    {
        return m_monitoring;
    }

    bool isActive(void)
    {
        return m_active;
    }

    void onValidDataFailed(Arming * arming)
    {
        (void)arming;
        arming->setRxFailsafe(false);
        m_validRxDataFailedAt = timeMillis();
        if ((m_validRxDataFailedAt - m_validRxDataReceivedAt) >
                m_rxDataFailurePeriod) {
            m_rxLinkState = RXLINK_DOWN;
        }
    }

    void onValidDataReceived(Arming * arming)
    {
        m_validRxDataReceivedAt = timeMillis();
            if ((m_validRxDataReceivedAt - m_validRxDataFailedAt) >
                    m_rxDataRecoveryPeriod) {
                m_rxLinkState = RXLINK_UP;
                arming->setRxFailsafe(true);
            }
        }

        void reset(void)
        {
            m_rxDataFailurePeriod =
                PERIOD_RXDATA_FAILURE + 4 * MILLIS_PER_TENTH_SECOND;
            m_rxDataRecoveryPeriod =
                PERIOD_RXDATA_RECOVERY + 20 * MILLIS_PER_TENTH_SECOND;
            m_validRxDataReceivedAt = 0;
            m_validRxDataFailedAt = 0;
            m_throttleLowPeriod = 0;
            m_landingShouldBeFinishedAt = 0;
            m_receivingRxDataPeriod = 0;
            m_receivingRxDataPeriodPreset = 0;
            m_phase = IDLE;
            m_rxLinkState = RXLINK_DOWN;
        }        

        void startMonitoring(void)
        {
            m_monitoring = true;
        }

        void update( float * rcData, void * motorDevice, Arming * arming)
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

                switch (m_phase) {
                    case IDLE:
                        if (arming->isArmed()) {
                            // Track throttle command below minimum time
                            if (!throttleIsDown(rcData)) {
                                m_throttleLowPeriod =
                                    timeMillis() + 100 * MILLIS_PER_TENTH_SECOND;
                            }
                            if (0 == SWITCH_MODE_KILL) {
                                activate();
                                m_phase = LANDED;      
                                m_receivingRxDataPeriodPreset =
                                    PERIOD_OF_1_SECONDS();    
                                // require 1 seconds of valid rxData
                                reprocessState = true;
                            } else if (!receivingRxData) {
                                if (timeMillis() > m_throttleLowPeriod
                                   ) {
                                    activate();

                                    // skip auto-landing procedure
                                    m_phase = LANDED;      
                                    m_receivingRxDataPeriodPreset =
                                        PERIOD_OF_3_SECONDS(); 
                                    // require 3 seconds of valid rxData
                                } else {
                                    m_phase = RX_LOSS_DETECTED;
                                }
                                reprocessState = true;
                            }
                        } else {
                            m_throttleLowPeriod = 0;
                        }
                        break;

                    case RX_LOSS_DETECTED:
                        if (receivingRxData) {
                            m_phase = RX_LOSS_RECOVERED;
                        } else {
                            // Drop the craft
                            activate();

                            // skip auto-landing procedure
                            m_phase = LANDED;      
                            break;
                        }
                        reprocessState = true;
                        break;

                    case LANDING:
                        break;
                    case LANDED:
                        arming->disarm(motorDevice);
                        m_receivingRxDataPeriod = timeMillis() +
                            m_receivingRxDataPeriodPreset; // set required
                        m_phase = RX_LOSS_MONITORING;
                        reprocessState = true;
                        break;

                    case RX_LOSS_MONITORING:
                        if (receivingRxData) {
                            if (timeMillis() > m_receivingRxDataPeriod) {
                                if (!arming->isArmed()) {
                                    m_phase = RX_LOSS_RECOVERED;
                                    reprocessState = true;
                                }
                            }
                        } else {
                            m_receivingRxDataPeriod = timeMillis() +
                                m_receivingRxDataPeriodPreset; 
                        }
                        break;

                    case RX_LOSS_RECOVERED:
                        m_throttleLowPeriod = timeMillis() + 100 *
                            MILLIS_PER_TENTH_SECOND;
                        m_phase = IDLE;
                        m_active = false;
                        reprocessState = true;
                        break;

                    default:
                        break;
                }
            } while (reprocessState);        
        }

}; // class Failsafe
