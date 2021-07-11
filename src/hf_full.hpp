/*
   Hackflight algorithm for real vehicles

   Supports adding serial communications tasks

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include <RFT_full.hpp>

#include "serialtask.hpp"

namespace hf {

    class Hackflight : public rft::RFT {

        public:

            void begin(rft::Board * board, Receiver * receiver, rft::Actuator * actuator)
            {
                rft::RFT::begin(board, receiver, actuator);
            }

            void update(rft::Board * board, Receiver * receiver, rft::Actuator * actuator, State * state)
            {
                rft::RFT::update(board, receiver, actuator, state);
            }

            void addSerialTask(SerialTask * task)
            {
                rft::RFT::addSerialTask(task);
            }

    }; // class Hackflight

}  // namespace hf
