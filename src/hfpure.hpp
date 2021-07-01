/*
   Hackflight core algorithm for real or simulated vehicles

   Copyright (c) 2021 Simon D. Levy

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
#include "pidcontroller.hpp"

namespace hf {

    class HackflightPure : public rft::RFT {

        protected:

            // Vehicle state
            State _state;

        public:

            HackflightPure(rft::Board * board, Receiver * receiver, rft::Actuator * actuator)
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
            }

            void update(void)
            {
                RFT::update();
            }

    }; // class HackflightPure

} // namespace hf
