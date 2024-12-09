/*

   SBUS implementation of Receiver interface

   Copyright (C) 2024 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#pragma once

// Bolderflight's SBUS library
#include <sbus.h>

#include <rx.hpp>

namespace hf {

    class SbusReceiver : public Receiver {

        public:

            void begin()
            {
                _sbus.Begin();
            }

            void read(uint16_t channels[6], bool & gotFailsafe) 
            {
                if (_sbus.Read()) {

                    const auto data = _sbus.data();

                    if (/*data.lost_frame ||*/ data.failsafe) {

                        gotFailsafe = true;
                    }

                    else {
                        channels[0] = data.ch[0];
                        channels[1] = data.ch[1];
                        channels[2] = data.ch[2];
                        channels[3] = data.ch[3];
                        channels[4] = data.ch[4];
                        channels[5] = data.ch[5];
                    }
                }
            }

            uint16_t minval() 
            {
                return 172;
            }

            uint16_t maxval() 
            {
                return 1811;
            }

        private:

            bfs::SbusRx _sbus = bfs::SbusRx(&Serial5);
    };
}
