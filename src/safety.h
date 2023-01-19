/*
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

#include "core/vstate.h"
#include "esc.h"
#include "imu.h"
#include "receiver.h"

class Safety {

    private:

        static constexpr float MAX_ARMING_ANGLE = 25;

        bool m_accDoneCalibrating;
        bool m_angleOkay;
        bool m_gyroDoneCalibrating;
        bool m_isArmed;
        bool m_ledOn;

    public:

        static const uint8_t  STARTUP_BLINK_LED_REPS  = 10;
        static const uint32_t STARTUP_BLINK_LED_DELAY = 50;

        typedef enum {

            OFF,
            ON,
            BLINK

        } state_e;

        typedef enum {

            LED_UNCHANGED,
            LED_TURN_ON,
            LED_TURN_OFF

        } ledChange_e;

        state_e state;

        uint32_t timer;

        void setTimer(const uint32_t usec)
        {
            timer = usec + 500000;
        }

        void disable(void)
        {
            state = OFF;
        }

        void blink(void)
        {
            state = BLINK;
        }

        // Avoid repeated degrees-to-radians conversion
        const float maxAngle = Imu::deg2rad(MAX_ARMING_ANGLE);

        bool gotFailsafe;
        bool haveSignal;
        bool switchOkay;
        bool throttleIsDown;

        void attemptToArm(Receiver & receiver, Esc * esc, const uint32_t usec)
        {
            static bool _doNotRepeat;

            if (receiver.aux1IsSet()) {

                if (ready()) {

                    if (m_isArmed) {
                        return;
                    }

                    if (!esc->isReady(usec)) {
                        return;
                    }

                    m_isArmed = true;
                }

            } else {

                disarm(esc);
            }

            if (!(m_isArmed || _doNotRepeat || !ready())) {
                _doNotRepeat = true;
            }
        }

        void disarm(Esc * esc)
        {
            if (m_isArmed) {
                esc->stop();
            }
            m_isArmed = false;
        }

        bool isArmed(void)
        {
            return m_isArmed;
        }

        bool ready(void)
        {
            return 
                m_accDoneCalibrating &&
                m_angleOkay &&
                !gotFailsafe &&
                haveSignal &&
                m_gyroDoneCalibrating &&
                switchOkay &&
                throttleIsDown;
        }

        void updateFromImu(Imu & imu, VehicleState & vstate)
        {
            const auto imuIsLevel =
                fabsf(vstate.phi) < maxAngle && fabsf(vstate.theta) < maxAngle;

            m_angleOkay = imuIsLevel;

            m_gyroDoneCalibrating = !imu.gyroIsCalibrating();

            m_accDoneCalibrating = true; // XXX
        }

        auto updateFromReceiver(
                Receiver * receiver,
                Esc * esc,
                const uint32_t usec) -> ledChange_e
        {
            ledChange_e ledChange = LED_UNCHANGED;

            if (receiver->getState() == Receiver::STATE_UPDATE) {
                attemptToArm(*receiver, esc, usec);
            }

            else  if (receiver->getState() == Receiver::STATE_CHECK) {

                if (isArmed()) {

                    if (!receiver->hasSignal() && haveSignal) {
                        gotFailsafe = true;
                        disarm(esc);
                    }
                    else {
                        ledChange = LED_TURN_ON;
                    }
                } 
                else {

                    throttleIsDown = receiver->throttleIsDown();

                    // If arming is disabled and the ARM switch is on
                    if (!ready() && receiver->aux1IsSet()) {
                        switchOkay = false;
                    } else if (!receiver->aux1IsSet()) {
                        switchOkay = true;
                    }

                    if (!ready()) {
                        blink();
                    } else {
                        disable();
                    }

                    if ((int32_t)(usec - timer) < 0) {
                        return ledChange;
                    }

                    switch (state) {
                        case OFF:
                            ledChange = LED_TURN_OFF;
                            break;
                        case ON:
                            ledChange = LED_TURN_ON;
                            break;
                        case BLINK:
                            m_ledOn = !m_ledOn;
                            ledChange = m_ledOn ? LED_TURN_ON : LED_TURN_OFF;
                            break;
                    }

                    setTimer(usec);
                }

                haveSignal = receiver->hasSignal();
            }

            return ledChange;
        }

 }; // class Safety
