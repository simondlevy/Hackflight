/*
   Hackflight core algorithm for real or simulated vehicles

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include <RFT_pure.hpp>
#include <RFT_board.hpp>
#include <RFT_actuator.hpp>

#include "state.hpp"
#include "receiver.hpp"

namespace hf {

    class Hackflight : public rft::RFTPure {

        public:

            void begin(rft::Board * board, Receiver * receiver, rft::Actuator * actuator)
            {
                rft::RFTPure::begin(board, receiver, actuator);
            }

            void update(rft::Board * board, Receiver * receiver, rft::Actuator * actuator, State * state)
            {
                rft::RFTPure::update(board, receiver, actuator, state);
            }

    }; // class HackflightPure

} // namespace hf
