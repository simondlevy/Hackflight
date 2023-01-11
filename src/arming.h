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

    private:

        bool readyToArm(void)
        {
            return 
                m_accDoneCalibrating &&
                m_angleOkay &&
                !m_gotFailsafe &&
                m_haveSignal &&
                m_gyroDoneCalibrating &&
                m_switchOkay &&
                m_throttleIsDown;
        }

        Esc  * m_esc;
        Led  * m_led;

        bool m_accDoneCalibrating;
        bool m_angleOkay;
        bool m_gotFailsafe;
        bool m_gyroDoneCalibrating;
        bool m_haveSignal;
        bool m_isArmed;
        bool m_switchOkay;
        bool m_throttleIsDown;

        void disarm(void)
        {
            if (m_isArmed) {
                m_esc->stop();
            }

            m_isArmed = false;
        }

    public:

        Arming(Led & led)
        {
            m_led = &led;
        }

        void begin(Esc * esc)
        {
            m_esc = esc;
        }

        void updateFromImu(const bool imuIsLevel, const bool gyroIsCalibrating)
        {
            m_angleOkay = imuIsLevel;

            m_gyroDoneCalibrating = !gyroIsCalibrating;

            m_accDoneCalibrating = true; // XXX
        }

        bool isArmed(void)
        {
            return m_isArmed;
        }

        // Called by Receiver
        void attempt(const uint32_t usec, const bool aux1IsSet)
        {
            static bool _doNotRepeat;

            if (aux1IsSet) {

                if (readyToArm()) {

                    if (m_isArmed) {
                        return;
                    }

                    if (!m_esc->isReady(usec)) {
                        return;
                    }

                    m_isArmed = true;
                }

            } else {

                if (m_isArmed) {
                    disarm();
                    m_isArmed = false;
                }
            }

            if (!(m_isArmed || _doNotRepeat || !readyToArm())) {
                _doNotRepeat = true;
            }
        }

        void updateFromReceiver(
                const bool throttleIsDown, const bool aux1IsSet, const bool haveSignal)
        {
            if (m_isArmed) {

                if (!haveSignal && m_haveSignal) {
                    m_gotFailsafe = true;
                    disarm();
            }
            else {
                m_led->set(true);
            }
        } else {

            m_throttleIsDown = throttleIsDown;

            // If arming is disabled and the ARM switch is on
            if (!readyToArm() && aux1IsSet) {
                m_switchOkay = false;
            } else if (!aux1IsSet) {
                m_switchOkay = true;
            }

            if (!readyToArm()) {
                m_led->warningFlash();
            } else {
                m_led->warningDisable();
            }

            m_led->warningUpdate();
        }

        m_haveSignal = haveSignal;
    }


}; // class Arming
