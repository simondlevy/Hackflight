/*
   RFT class supporting serial communication

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "RFT_pure.hpp"

#include "HF_serialtask.hpp"
#include "HF_board.hpp"
#include "HF_mixer.hpp"
#include "HF_state.hpp"

namespace rft {

    class RFTFull : public RFTPure {

        private:

            // Serial tasks
            hf::SerialTask * _serial_tasks[10] = {};
            uint8_t _serial_task_count = 0;

        protected:

            RFTFull(hf::Board * board, hf::Receiver * receiver, hf::Mixer * mixer)
                : RFTPure(board, receiver, mixer)
            {
                _serial_task_count = 0;
            }

            void update(hf::State * state)
            {
                RFTPure::update(state);

                // Update serial tasks
                for (uint8_t k=0; k<_serial_task_count; ++k) {
                    _serial_tasks[k]->update(_board, _mixer, state);
                }
            }

            void addSerialTask(hf::SerialTask * task)
            {
                _serial_tasks[_serial_task_count++] = task;
            }

    }; // class RFT

} // namespace
