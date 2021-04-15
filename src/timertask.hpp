/*
   Abstract class for timer tasks

   Copyright (c) 2020 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "board.hpp"

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
