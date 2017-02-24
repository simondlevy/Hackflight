/*
   config.hpp : #define'd constants

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

// min, max interval in usec for motors, RX PWM
#define CONFIG_PWM_MIN  1000
#define CONFIG_PWM_MAX  2000

#define CONFIG_MAGNETIC_DECLINATION                 0

#define CONFIG_CALIBRATING_ACC_MSEC                 1400

#define CONFIG_RC_LOOPTIME_MSEC                     20
#define CONFIG_CALIBRATE_ACCTIME_MSEC               500
#define CONFIG_SMALL_ANGLE                          250  // tenths of a degree
#define CONFIG_ALTITUDE_UPDATE_MSEC                 25   // based on accelerometer low-pass filter
