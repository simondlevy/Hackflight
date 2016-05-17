/*
   rc.hpp : RC receiver class header

   Adapted from https://github.com/multiwii/baseflight/blob/master/src/mw.c

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

// For logical combinations of stick positions (low, center, high)
#define ROL_LO (1 << (2 * ROLL))
#define ROL_CE (3 << (2 * ROLL))
#define ROL_HI (2 << (2 * ROLL))
#define PIT_LO (1 << (2 * PITCH))
#define PIT_CE (3 << (2 * PITCH))
#define PIT_HI (2 << (2 * PITCH))
#define YAW_LO (1 << (2 * YAW))
#define YAW_CE (3 << (2 * YAW))
#define YAW_HI (2 << (2 * YAW))
#define THR_LO (1 << (2 * THROTTLE))
#define THR_CE (3 << (2 * THROTTLE))
#define THR_HI (2 << (2 * THROTTLE))

#define CONFIG_RC_EXPO_8                            65
#define CONFIG_RC_RATE_8                            90
#define CONFIG_MIDRC                                1490
#define CONFIG_THR_MID_8                            50
#define CONFIG_THR_EXPO_8                           0
#define CONFIG_MINCHECK                             1100
#define CONFIG_MAXCHECK                             1900
#define CONFIG_MINTHROTTLE                          990
#define CONFIG_MAXTHROTTLE                          2010

#define PITCH_LOOKUP_LENGTH    7
#define THROTTLE_LOOKUP_LENGTH 12

extern "C" {

    class RC {

        private:

            Board * _board;

            int16_t dataAverage[8][4];
            uint8_t commandDelay;                               // cycles since most recent movement
            int32_t averageIndex;
            int16_t lookupPitchRollRC[PITCH_LOOKUP_LENGTH];     // lookup table for expo & RC rate PITCH+ROLL
            int16_t lookupThrottleRC[THROTTLE_LOOKUP_LENGTH];   // lookup table for expo & mid THROTTLE

        public:

            uint8_t sticks;            // holds stick positions for command combos
            void init(Board * board);

            int16_t command[4];
            int16_t data[RC_CHANS];

            void update(void);

            bool changed(void);

            void computeExpo(void);

            uint8_t auxState(void);

            bool throttleIsDown(void);
    };
}
