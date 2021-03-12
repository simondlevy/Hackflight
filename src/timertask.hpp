/*
   Abstract class for timer tasks

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "board.hpp"
#include "debugger.hpp"

namespace hf {

    class TimerTask {

        private:

            float _period = 0;
            float _time = 0;

        protected:

            Board * _board = NULL;

            TimerTask(float freq)
            {
                _period = 1 / freq;
                _time = 0;
            }

            void begin(Board * board)
            {
                _board = board;
            }

            virtual void doTask(void) = 0;

        public:

            void update(void)
            {
                float time = _board->getTime();

                if ((time - _time) > _period)
                {
                    doTask();
                    _time = time;
                }
            }

    };  // TimerTask

} // namespace hf
