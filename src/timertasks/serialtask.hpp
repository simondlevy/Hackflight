/*
   Timer task for serial comms

   Copyright (C) 2020 Simon D. Levy

   MIT License
 */

#pragma once

#include "timertask.hpp"
#include "mspparser.hpp"

#include <RFT_actuator.hpp>
#include <RFT_board.hpp>

namespace hf {

    class SerialTask : public TimerTask, public MspParser {

        friend class Hackflight;

        private:

            static constexpr float FREQ = 66;

            rft::Actuator * _actuator = NULL;
            Receiver * _receiver = NULL;
            State  * _state = NULL;

            void (*_actuatorfun)(State * state, rft::Actuator * actuator);

            static void _actuatorfunFull(State * state, rft::Actuator * actuator)
            {
                if (!state->armed) {
                    actuator->runDisarmed();
                }
            }

            static void _actuatorfunProxy(State * state, rft::Actuator * actuator)
            {
                (void)state;
                (void)actuator;
            }

            void _begin(rft::Board * board, State * state, Receiver * receiver) 
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
                _actuatorfun(_state, _actuator);
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
                heading = -_state->x[State::PSI]; // NB: Angle negated for remote visualization
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

            void handle_ACTUATOR_TYPE_Request(uint8_t & type) override
            {
                type = 0; // XXX _actuator->getType();
            }

            virtual void handle_ATTITUDE_RADIANS_Request(float & roll, float & pitch, float & yaw) override
            {
                roll  = _state->x[State::PHI];
                pitch = _state->x[State::THETA];
                yaw   = _state->x[State::PSI];
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

            void begin(rft::Board * board, State * state, Receiver * receiver) 
            {
                _begin(board, state, receiver);
                _actuatorfun = _actuatorfunProxy;
            }

            void begin(rft::Board * board, State * state, Receiver * receiver, rft::Actuator * actuator) 
            {
                begin(board, state, receiver);
                _actuator = actuator;
                _actuatorfun = _actuatorfunFull;
            }

    };  // SerialTask

} // namespace hf
