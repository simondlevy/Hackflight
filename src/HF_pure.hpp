/*
   Hackflight core algorithm for real or simulated vehicles

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "RFT_pure.hpp"
#include "RFT_board.hpp"

#include "HF_mixer.hpp"
#include "HF_state.hpp"
#include "HF_receiver.hpp"

namespace hf {

    class HackflightPure : public rft::RFTPure {

        protected:

            State _state = {};

        public:

            HackflightPure(rft::Board * board,
                       Receiver * receiver,
                       Mixer * mixer)
                : rft::RFTPure(board, receiver, mixer)
            {
            }

            void begin(bool armed=false)
            {
                // Supports automatic arming for simulator
                _state.armed = armed;

                rft::RFTPure::begin();
            }

            void update(void)
            {
                rft::RFTPure::update(&_state);
            }

    }; // class Hackflight

} // namespace hf
