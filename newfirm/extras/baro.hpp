/*
   baro.hpp : Baro class header

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

#ifdef __arm__
extern "C" {
#endif

    class Baro {
        
        private:

            bool avail;

            static const int TABLE_SIZE = 21;

            uint32_t pressureSum;
            int32_t  historyTable[TABLE_SIZE];
            int      historyIdx;

        public:

            void init(void);

            bool available(void);

            void update(void);

            int32_t getAltitude(void);
    };

#ifdef __arm__
} // extern "C"
#endif
