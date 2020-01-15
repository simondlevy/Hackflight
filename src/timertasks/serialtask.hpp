/*
   Timer task for serial comms

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

#include "timertask.hpp"
#include "board.hpp"
#include "mspparser.hpp"

namespace hf {

    class SerialTask : public TimerTask, public MspParser {

        private:

            static constexpr float FREQ = 66;

            Board * _board;

        protected:

            virtual void doTask(void) override
            {
                while (_board->serialAvailableBytes() > 0) {

                    if (MspParser::parse(_board->serialReadByte())) {
                        _board->reboot(); // parser returns true when reboot requested
                    }
                }

                while (MspParser::availableBytes() > 0) {
                    _board->serialWriteByte(MspParser::readByte());
                }
            }

        public:

            SerialTask(Board * board) 
                : TimerTask(FREQ)
            {
                _board = board;
            }


    };  // SerialTask

} // namespace hf
