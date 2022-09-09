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

#include "esc.h"
#include "led.h"

class Arming {

    friend class AttitudeTask;
    friend class Hackflight;
    friend class Msp;
    friend class Receiver;
    friend class SoftQuatImu;

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

    Esc  * m_esc;
    Led  * m_led;

    bool m_acc_done_calibrating;
    bool m_angle_okay;
    bool m_switch_okay;
    bool m_gyro_done_calibrating;
    bool m_is_armed;
    bool m_rx_failsafe_okay;
    bool m_throttle_is_down;

    void begin(Esc * esc, Led * led)
    {
        m_led = led;
        m_esc = esc;
    }

    void disarm(void)
    {
        if (m_is_armed) {
            m_esc->stop();
        }

        m_is_armed = false;
    }

    bool isArmed(void)
    {
        return m_is_armed;
    }

    void updateImuStatus(const bool imuIsLevel, const bool gyroIsCalibrating)
    {
        m_angle_okay = imuIsLevel;

        m_gyro_done_calibrating = !gyroIsCalibrating;

        m_acc_done_calibrating = true;
    }

    // Called by Receiver
    void attempt(const uint32_t currentTimeUs, const bool aux1IsSet)
    {
        static bool _doNotRepeat;

        if (aux1IsSet) {

            if (readyToArm()) {

                if (m_is_armed) {
                    return;
                }

                if (!m_esc->isReady(currentTimeUs)) {
                    return;
                }

                m_is_armed = true;
            }

        } else {

            if (m_is_armed) {
                disarm();
                m_is_armed = false;
            }
        }

        if (!(m_is_armed || _doNotRepeat || !readyToArm())) {
            _doNotRepeat = true;
        }
    }

    void updateReceiverStatus(const bool throttleIsDown, const bool aux1IsSet)
    {
        if (m_is_armed) {
            m_led->set(true);
        } else {

            m_throttle_is_down = throttleIsDown;

            // If arming is disabled and the ARM switch is on
            if (!readyToArm() && aux1IsSet) {
                m_switch_okay = false;
            } else if (!aux1IsSet) {
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
