/*
   Datatype declarations for vehicle state

   Copyright (c) 2018 Simon D. Levy

   MIT License
   */

#pragma once

namespace hf {

    class State {

        public:

            bool armed;
            bool failsafe;

            // See Bouabdallah et al. (2004)
            enum {X, DX, Y, DY, Z, DZ, PHI, DPHI, THETA, DTHETA, PSI, DPSI, SIZE};

            float x[SIZE];

    }; // class State

} // namespace hf
