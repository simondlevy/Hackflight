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

#include "led.h"
#include "motors.h"
#include "sticks.h"

class Arming {

    friend class Failsafe;
    friend class FusionImu;
    friend class Hackflight;
    friend class Receiver;
    friend class MspTask;

    bool readyToArm(void)
    {
        return 
            m_acc_done_calibrating &&
            m_angle_okay &&
            m_switch_okay &&
            m_gyro_done_calibrating &&
            m_rx_failsafe_okay &&
            m_throttle_is_down;
    }

    static bool rxAux1IsSet(float raw[])
    {
        return raw[4] > 1200;
    }

    Led  * m_led;

    bool m_acc_done_calibrating;
    bool m_angle_okay;
    bool m_switch_okay;
    bool m_gyro_done_calibrating;
    bool m_is_armed;
    bool m_rx_failsafe_okay;
    bool m_throttle_is_down;

    void check(
            void * motorDevice,
            uint32_t currentTimeUs,
            float raw[],
            bool imuIsLevel,
            bool calibrating)
    {
        static bool _doNotRepeat;

        if (rxAux1IsSet(raw)) {

            updateStatus(raw, imuIsLevel, calibrating);

            if (readyToArm()) {

                if (m_is_armed) {
                    return;
                }

                if (!motorDevIsReady(currentTimeUs)) {
                    return;
                }

                m_is_armed = true;

            }

        } else {

            if (m_is_armed) {
                disarm(motorDevice);
                m_is_armed = false;
            }
        }

        if (!(m_is_armed || _doNotRepeat || !readyToArm())) {
            _doNotRepeat = true;
        }
    }

    void disarm(void * motorDevice)
    {
        if (m_is_armed) {
            motorDevStop(motorDevice);
        }

        m_is_armed = false;
    }

    bool isArmed(void)
    {
        return m_is_armed;
    }

    void updateStatus( float raw[], bool imuIsLevel, bool calibrating)
    {
        if (m_is_armed) {
            m_led->set(true);
        } else {

            m_throttle_is_down = throttleIsDown(raw);

            m_angle_okay = imuIsLevel;

            m_gyro_done_calibrating = !calibrating;

            m_acc_done_calibrating = true;

            // If arming is disabled and the ARM switch is on
                if (!readyToArm() && rxAux1IsSet(raw)) {
                    m_switch_okay = false;
                } else if (!rxAux1IsSet(raw)) {
                    m_switch_okay = true;
                }

                if (!readyToArm()) {
                    m_led->warningFlash();
                } else {
                    m_led->warningDisable();
                }

                m_led->warningUpdate();
            }
        }

        void setRxFailsafe(bool okay)
        {
            m_rx_failsafe_okay= okay;
        }

}; // class Arming
