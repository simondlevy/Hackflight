/*
   Hackflight core algorithm for real or simulated vehicles

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "RFT_pure.hpp"
#include "RFT_board.hpp"
#include "RFT_actuator.hpp"

#include "HF_state.hpp"
#include "HF_receiver.hpp"

namespace hf {

    class Hackflight : public rft::RFTPure {

        protected:

            State _state = {};

        public:

            Hackflight(rft::Board * board,
                       Receiver * receiver,
                       rft::Actuator * actuator)
                : rft::RFTPure(board, receiver, actuator)
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
