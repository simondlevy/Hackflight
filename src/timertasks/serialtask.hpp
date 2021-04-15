/*
   Timer task for serial comms

   Copyright (c) 2020 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "timertask.hpp"
#include "board.hpp"
#include "mspparser.hpp"
#include "mixer.hpp"

namespace hf {

    class SerialTask : public TimerTask, public MspParser {

        friend class Hackflight;

        private:

            static constexpr float FREQ = 66;

            Mixer    * _mixer = NULL;
            Receiver * _receiver = NULL;
            state_t  * _state = NULL;

            void _begin(Board * board, state_t * state, Receiver * receiver) 
            {
                TimerTask::begin(board);

                MspParser::begin();

                _state = state;
                _receiver = receiver;
             }

        protected:

            // TimerTask overrides -------------------------------------------------------

            virtual void doTask(void) override
            {
                while (_board->serialAvailableBytes() > 0) {

                    MspParser::parse(_board->serialReadByte());
                }

                while (MspParser::availableBytes() > 0) {
                    _board->serialWriteByte(MspParser::readByte());
                }

                // Support motor testing from GCS
                if (!_state->armed) {
                    _mixer->runDisarmed();
                }
            }

            // MspParser overrides -------------------------------------------------------

            virtual void handle_STATE_Request(float & altitude, float & variometer, float & positionX, float & positionY, 
                    float & heading, float & velocityForward, float & velocityRightward) override
            {
                // XXX Use only heading for now
                altitude = 0;
                variometer = 0;
                positionX = 0;
                positionY = 0;
                heading = -_state->x[STATE_PSI]; // NB: Angle negated for remote visualization
                velocityForward = 0;
                velocityRightward = 0;
            }
 
            virtual void handle_SET_ARMED(uint8_t  flag) override
            {
                if (flag) {  // got arming command: arm only if throttle is down
                    if (_receiver->throttleIsDown()) {
                        _state->armed = true;
                    }
                }
                else {          // got disarming command: always disarm
                    _state->armed = false;
                }
            }

            virtual void handle_RC_NORMAL_Request(float & c1, float & c2, float & c3, float & c4, float & c5, float & c6) override
            {
                c1 = _receiver->getRawval(0);
                c2 = _receiver->getRawval(1);
                c3 = _receiver->getRawval(2);
                c4 = _receiver->getRawval(3);
                c5 = _receiver->getRawval(4);
                c6 = _receiver->getRawval(5);
            }

            virtual void handle_ATTITUDE_RADIANS_Request(float & roll, float & pitch, float & yaw) override
            {
                roll  = _state->x[STATE_PHI];
                pitch = _state->x[STATE_THETA];
                yaw   = _state->x[STATE_PSI];
            }

            virtual void handle_SET_MOTOR_NORMAL(float  m1, float  m2, float  m3, float  m4) override
            {
                _mixer->motorsDisarmed[0] = m1;
                _mixer->motorsDisarmed[1] = m2;
                _mixer->motorsDisarmed[2] = m3;
                _mixer->motorsDisarmed[3] = m4;
            }

            SerialTask(void)
                : TimerTask(FREQ)
            {
            }

            void begin(Board * board, state_t * state, Receiver * receiver, Mixer * mixer) 
            {
                TimerTask::begin(board);
                _state = state;
                _receiver = receiver;
                _mixer = mixer;
            }

    };  // SerialTask

} // namespace hf
