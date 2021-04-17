/*
   Hackflight state info

   Copyright (c) 2018 Simon D. Levy

   MIT License
   */

#pragma once

namespace hf {

    enum {
        AXIS_ROLL = 0,
        AXIS_PITCH, 
        AXIS_YAW
    };

    // See Bouabdallah et al. (2004)
    enum {
        STATE_X = 0,
        STATE_DX,
        STATE_Y,
        STATE_DY,
        STATE_Z,
        STATE_DZ,
        STATE_PHI,
        STATE_DPHI,
        STATE_THETA,
        STATE_DTHETA,
        STATE_PSI,
        STATE_DPSI,
        STATE_SIZE
    };

    typedef struct {

        float x[STATE_SIZE];

        bool armed;
        bool failsafe;

    } state_t;

} // namespace hf
