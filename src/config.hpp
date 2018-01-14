/*
   config.hpp : configuration values (PID etc.) for a specific vehicle

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>

namespace hf {





//=========================================================================
// Barometer config
//=========================================================================

struct BarometerConfig {

    float                noiseLpf         = 0.5f;
    float                velocityBound    = 300.0f;
    float                velocityDeadband = 10.0f;
    static const uint8_t HISTORY_SIZE     = 48;
};

//=========================================================================
// Accelerometer config (for atltitude hold)
//=========================================================================

struct AccelerometerConfig {

    // These probably don't need to be changed
    float lpfCutoff  = 5.00f;
    float lpfFactor  = 0.25f;
    float deadband   = 0.02f;
};

//=========================================================================
// Altitude-hold config
//=========================================================================

struct AltitudeConfig {

    // PIDs are in model.hpp

    // Bounds
    float pDeadband = 0.01;
    float dDeadband = 0.1;
    float pidMax    = 4.0;
    float pErrorMax = 1.0;
    float iErrorMax = 8.0;

    // Barometer
    BarometerConfig baro;

    // Acceleromter
    AccelerometerConfig accel;

    // Complementry filter for accel/baro
    float    cfAlt                  = 0.965f;
    float    cfVel                  = 0.985f;

    // Keeps PID adjustment inside range
    float throttleMargin            = 0.15;
};

//=========================================================================
// Receiver config
//=========================================================================

struct ReceiverConfig {

    float margin        = 0.1f;
    float pitchRollExpo = 0.65f;
    float pitchRollRate = 0.90f;
    float throttleMid   = 0.50f;
    float throttleExpo  = 0.20f;

    bool  headless      = false;

    static const uint8_t CHANNELS = 8;
};

//=========================================================================
// initialization config
//=========================================================================

struct InitConfig {

    uint32_t delayMilli    = 100;
    uint32_t ledFlashMilli = 1000;
    uint32_t ledFlashCount = 20;
};

//=========================================================================
// all config
//=========================================================================

struct Config {
    AltitudeConfig   altitude;
    ReceiverConfig   receiver;
    InitConfig       init;
};

//=========================================================================
// shared constants
//=========================================================================

enum {
    DEMAND_THROTTLE, // T
    DEMAND_ROLL,     // A
    DEMAND_PITCH,    // E
    DEMAND_YAW,      // R
    DEMAND_AUX1,
    DEMAND_AUX2,
    DEMAND_AUX3,
    DEMAND_AUX4
};

} // namespace
