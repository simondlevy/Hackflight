/*
   mixer.hpp : Mixer class header

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

#define CONFIG_YAW_DIRECTION     1

#ifdef __arm__
extern "C" {
#endif

    class Mixer {

        private:

            Board     * _board;
            RC        * _rc;
            Stabilize * _stabilize;
        
        public:

            int16_t  motorsDisarmed[4];

            void init(Board * board, RC * rc, Stabilize * stabilize);

            void update(bool armed);
    };

#ifdef __arm__
} // extern "C"
#endif
