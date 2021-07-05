/*
   Hackflight core algorithm for real or simulated vehicles

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include <RoboFirmwareToolkit.hpp>
#include <RFT_board.hpp>
#include <RFT_actuator.hpp>

#include "state.hpp"
#include "receiver.hpp"

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

    }; // class HackflightPure

} // namespace hf
