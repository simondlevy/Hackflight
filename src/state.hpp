/*
   Hackflight state class

   Copyright (c) 2018 Simon D. Levy

   MIT License
   */

#pragma once

namespace hf {

    class State {

        public:

            enum {AXIS_ROLL, AXIS_PITCH, AXIS_YAW};

            // See Bouabdallah et al. (2004)
            enum {X, DX, Y, DY, Z, DZ, PHI, DPHI, THETA, DTHETA, PSI, DPSI, SIZE};

            float x[SIZE];

            bool armed;
            bool failsafe;

    }; // class State


} // namespace hf
