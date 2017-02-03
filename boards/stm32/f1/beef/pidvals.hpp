/*
   pidvals.hpp : PID values for a specific vehicle

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

// Level (accelerometer)
static const uint8_t CONFIG_LEVEL_P          = 40;
static const uint8_t CONFIG_LEVEL_I          = 10;

// Rate (gyro): P must be positive
static const uint8_t CONFIG_RATE_PITCHROLL_P = 36;
static const uint8_t CONFIG_RATE_PITCHROLL_I = 30;
static const uint8_t CONFIG_RATE_PITCHROLL_D = 23;

// Yaw: P must be positive
static const uint8_t CONFIG_YAW_P            = 85;
static const uint8_t CONFIG_YAW_I            = 45;

// For altitude hover
#define CONFIG_HOVER_ALT_P  120
#define CONFIG_HOVER_ALT_I  45
#define CONFIG_HOVER_ALT_D  1


