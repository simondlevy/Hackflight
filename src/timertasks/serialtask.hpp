/*
   Timer task for serial comms

   Copyright (c) 2020 Simon D. Levy

   MIT License
 */

#pragma once

#include <RFT_board.hpp>
#include <RFT_timertask.hpp>

#include "mspparser.hpp"
#include "mixer.hpp"

namespace hf {

    class SerialTask : public rft::TimerTask, public MspParser {

        friend class Hackflight;

        private:

            static constexpr float FREQ = 66;

            Mixer    * _mixer = NULL;
            Receiver * _receiver = NULL;
            State  * _state = NULL;

            void _begin(rft::Board * board, State * state, Receiver * receiver) 
            {
                rft::TimerTask::begin(board);

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
                roll  = _state->x[State::PHI];
                pitch = _state->x[State::THETA];
                yaw   = _state->x[State::PSI];
            }

            virtual void handle_SET_MOTOR_NORMAL(float  m1, float  m2, float  m3, float  m4) override
            {
                _mixer->motorsDisarmed[0] = m1;
                _mixer->motorsDisarmed[1] = m2;
                _mixer->motorsDisarmed[2] = m3;
                _mixer->motorsDisarmed[3] = m4;
            }

            SerialTask(void)
                : rft::TimerTask(FREQ)
            {
            }

            void begin(rft::Board * board, State * state, Receiver * receiver, Mixer * mixer) 
            {
                rft::TimerTask::begin(board);
                _state = state;
                _receiver = receiver;
                _mixer = mixer;
            }

    };  // SerialTask

} // namespace hf
