/*
   Hackflight algorithm for real vehicles

   Supports adding serial communications tasks

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "RFT_full.hpp"

#include "HF_serialtask.hpp"

namespace hf {

    class HackflightFull: public rft::RFTFull {
        
        private:

            State _state = {};

        public:

            HackflightFull(rft::Board * board, Receiver * receiver, Mixer * mixer)
                : rft::RFTFull(board, receiver, mixer)
            {
            }

            void begin(void)
            {
                rft::RFTFull::begin();
            }

            void update(void)
            {
                rft::RFTFull::update(&_state);
            }

            void addSerialTask(SerialTask * task)
            {
                rft::RFTFull::addSerialTask(task);

                task->init((Receiver *)_olc, _mixer, &_state);
            }

    }; // class Hackflight

}  // namespace hf
