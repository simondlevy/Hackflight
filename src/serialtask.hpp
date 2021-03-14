/*
   Timer task for serial comms

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include <RFT_timertask.hpp>
#include <RFT_board.hpp>
#include <RFT_debugger.hpp>
#include <RFT_actuator.hpp>

#include "mavstate.hpp"
#include "parser.hpp"

namespace hf {

    class SerialTask : public rft::TimerTask, public Parser {

        friend class Hackflight;

        private:

            static constexpr float FREQ = 66;

            rft::Actuator * _actuator = NULL;
            rft::OpenLoopController * _olc = NULL;
            MavState  * _state = NULL;

            void _begin(rft::Board * board, MavState * state, rft::OpenLoopController * olc) 
            {
                TimerTask::begin(board);

                Parser::begin();

                _state = state;
                _olc = olc;
             }

        protected:

            // TimerTask overrides -------------------------------------------------------

            virtual void doTask(void) override
            {
                while (_board->serialAvailableBytes() > 0) {

                    Parser::parse(_board->serialReadByte());
                }

                while (Parser::availableBytes() > 0) {
                    _board->serialWriteByte(Parser::readByte());
                }

                // Support motor testing from GCS
                if (!_state->armed) {
                    _actuator->runDisarmed();
                }
            }

            // Parser overrides -------------------------------------------------------

            virtual void handle_Receiver_Request(float & c1, float & c2, float & c3, float & c4, float & c5, float & c6) override
            {
                c1 = _olc->getRawval(0);
                c2 = _olc->getRawval(1);
                c3 = _olc->getRawval(2);
                c4 = _olc->getRawval(3);
                c5 = _olc->getRawval(4);
                c6 = _olc->getRawval(5);
            }

            virtual void handle_ATTITUDE_RADIANS_Request(float & roll, float & pitch, float & yaw) override
            {
                roll  = _state->x[MavState::STATE_PHI];
                pitch = _state->x[MavState::STATE_THETA];
                yaw   = _state->x[MavState::STATE_PSI];
            }

            virtual void handle_SET_MOTOR_NORMAL(float  m1, float  m2, float  m3, float  m4) override
            {
                _actuator->setMotorDisarmed(0, m1);
                _actuator->setMotorDisarmed(1, m2);
                _actuator->setMotorDisarmed(2, m3);
                _actuator->setMotorDisarmed(3, m4);
            }

            SerialTask(void)
                : TimerTask(FREQ)
            {
            }

            void begin(rft::Board * board, rft::State * state, rft::OpenLoopController * olc, rft::Actuator * actuator) 
            {
                rft::TimerTask::begin(board);
                _state = (MavState *)state;
                _olc = olc;
                _actuator = actuator;
            }

    };  // SerialTask

} // namespace hf
