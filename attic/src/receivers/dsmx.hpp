/*

   DSMX implementation of Receiver interface

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

// Hackflight Receiver interface
#include <rx.hpp>

static Dsm2048 _dsm2048;


namespace hf {

    class DsmxReceiver : public Receiver {

        public:

            void begin()
            {
                Serial2.begin(115000);
            }

            void read(uint16_t channels[6], bool & gotFailsafe) 
            {
                gotFailsafe = false;

           }

            uint16_t minval() 
            {
                return 988;
            }

            uint16_t maxval() 
            {
                return 2011;
            }

    };
}
