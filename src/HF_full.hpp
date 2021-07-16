/*
   Hackflight algorithm for real vehicles

   Supports adding serial communications tasks

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include <RFT_full.hpp>

#include "HF_serialtask.hpp"

namespace hf {

    class Hackflight: public rft::RFT {
        
        private:

            State _state = {};

        public:

            Hackflight(rft::Board * board, Receiver * receiver, rft::Actuator * actuator)
                : rft::RFT(board, receiver, actuator)
            {
            }

            void begin(void)
            {
                rft::RFT::begin();
            }

            void update(void)
            {
                rft::RFT::update(&_state);
            }

            void addSerialTask(SerialTask * task)
            {
                rft::RFT::addSerialTask(task);

                task->init((Receiver *)_olc, _actuator, &_state);
            }

    }; // class Hackflight

}  // namespace hf
