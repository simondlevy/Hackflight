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
static const uint8_t CONFIG_LEVEL_P          = 90;  
static const uint8_t CONFIG_LEVEL_I          = 10;  

// Rate (gyro): P must be positive
static const uint8_t CONFIG_RATE_PITCHROLL_P = 33;  
static const uint8_t CONFIG_RATE_PITCHROLL_I = 30;  
static const uint8_t CONFIG_RATE_PITCHROLL_D = 23;  

// Yaw: P must be positive
static const uint8_t CONFIG_YAW_P            = 85;  
static const uint8_t CONFIG_YAW_I            = 45;  



// Level (accelerometer)
static const float CONFIG_LEVEL_P_f             = 9.0;
static const float CONFIG_LEVEL_I_f             = 0.010;

// Rate (gyro): P must be positive
static const float CONFIG_RATE_PITCHROLL_P_f    = 3.3;
static const float CONFIG_RATE_PITCHROLL_I_f    = 0.030;
static const float CONFIG_RATE_PITCHROLL_D_f    = 23;

// Yaw: P must be positive
static const float CONFIG_YAW_P_f               = 8.5;
static const float CONFIG_YAW_I_f               = 0.045;

// For altitude hover
#define CONFIG_HOVER_ALT_P  120
#define CONFIG_HOVER_ALT_I  45
#define CONFIG_HOVER_ALT_D  1


