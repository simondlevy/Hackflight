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

#ifdef __arm__
extern "C" {
#endif

    class Baro {
        
        private:

            bool avail;

            static const int TABLE_SIZE     = 21;
            static const int TABLE_SIZE_MAX = 48;

            uint32_t pressureSum;
            int32_t  histTable[TABLE_SIZE_MAX];
            int      histIdx;

            Board * _board;

        public:

            void init(Board * board);

            bool available(void);

            int32_t getAltitude(void);
    };

#ifdef __arm__
} // extern "C"
#endif
