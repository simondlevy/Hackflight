/*
   Timer task for serial comms

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "timertask.hpp"
#include "board.hpp"
#include "mspparser.hpp"
#include "debugger.hpp"
#include "actuator.hpp"
#include "states/mavstate.hpp"

namespace hf {

    class SerialTask : public TimerTask, public MspParser {

        friend class Hackflight;

        private:

            static constexpr float FREQ = 66;

            Actuator * _actuator = NULL;
            OpenLoopController * _olc = NULL;
            MavState  * _state = NULL;

            void _begin(Board * board, MavState * state, OpenLoopController * olc) 
            {
                TimerTask::begin(board);

                MspParser::begin();

                _state = state;
                _olc = olc;
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
                    _actuator->runDisarmed();
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
                heading = -_state->x[MavState::STATE_PSI]; // NB: Angle negated for remote visualization
                velocityForward = 0;
                velocityRightward = 0;
            }
 
            virtual void handle_OLC_Request(float & c1, float & c2, float & c3, float & c4, float & c5, float & c6) override
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

            void begin(Board * board, State * state, OpenLoopController * olc, Actuator * actuator) 
            {
                TimerTask::begin(board);
                _state = (MavState *)state;
                _olc = olc;
                _actuator = actuator;
            }

    };  // SerialTask

} // namespace hf
