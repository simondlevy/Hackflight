/*
   Hackflight core algorithm

   Copyright (c) 2020 Simon D. Levy

   MIT License
 */

#pragma once

#include <RoboFirmwareToolkit.hpp>
#include <RFT_board.hpp>
#include <RFT_openloop.hpp>
#include <RFT_actuator.hpp>
#include <rft_timertasks/closedlooptask.hpp>

#include "state.hpp"
#include "receiver.hpp"
#include "serialtask.hpp"
#include "pidcontroller.hpp"

namespace hf {

    class Hackflight : public rft::RFT {

        private:

            // Vehicle state
            State _state;

            // Serial timer task for GCS
            SerialTask _serialTask;

        public:

            Hackflight(rft::Board * board, Receiver * receiver, rft::Actuator * actuator)
                : rft::RFT(&_state, board, receiver, actuator)
            {  
            }

            void addPidController(PidController * pidController, uint8_t auxState=0) 
            {
                RFT::addClosedLoopController(pidController, auxState);
            }

            void begin(bool armed=false)
            {  
                // Initialize state
                memset(&_state.x, 0, sizeof(_state.x));

                RFT::begin(armed);

                // Initialize serial timer task
                _serialTask.begin(_board, &_state, _olc, _actuator);

            } // begin


            void update(void)
            {
                RFT::update();

                // Update serial comms task
                _serialTask.update();
            }

    }; // class Hackflight

} // namespace hf
