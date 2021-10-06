/*
   Hackflight class supporting safety checks and serial communication

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "HF_pure.hpp"

#include "HF_serialtask.hpp"
#include "HF_board.hpp"
#include "HF_mixer.hpp"
#include "HF_state.hpp"

namespace hf {

    class HackflightFull : public HackflightPure {

        private:

            // Serial tasks
            SerialTask * _serial_tasks[10] = {};
            uint8_t _serial_task_count = 0;

        public:

            HackflightFull(Board * board, Receiver * receiver, Mixer * mixer)
                : HackflightPure(board, receiver, mixer)
            {
                _serial_task_count = 0;
            }

            void update(void)
            {
                HackflightPure::update();

                // Update serial tasks
                for (uint8_t k=0; k<_serial_task_count; ++k) {
                    _serial_tasks[k]->update(_board, _mixer, &_state);
                }
            }

            void addSerialTask(SerialTask * task)
            {
                _serial_tasks[_serial_task_count++] = task;

                task->init(_receiver, _mixer, &_state);
            }

    }; // class HackflightFull

} // namespace
