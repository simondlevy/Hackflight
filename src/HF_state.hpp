/*
   Datatype declarations for vehicle state

   Copyright (c) 2018 Simon D. Levy

   MIT License
   */

#pragma once

#include "HF_filters.hpp"

namespace hf {

    class State {

        friend class HackflightPure;
        friend class HackflightFull;

        private:

            static constexpr float MAX_ARMING_ANGLE_DEGREES = 25;

            bool safeAngle(uint8_t axis)
            {
                return fabs(x[axis]) < Filter::deg2rad(MAX_ARMING_ANGLE_DEGREES);
            }

        public:

            bool armed = false;
            bool failsafe = false;

            // See Bouabdallah et al. (2004)
            enum {X, DX, Y, DY, Z, DZ, PHI, DPHI, THETA, DTHETA, PSI, DPSI, SIZE};

            float x[SIZE];

            State(bool start_armed=false)
            {
                armed = start_armed;
            }

            bool safeToArm(void)
            {
                return safeAngle(PHI) && safeAngle(THETA);
            }


    }; // class State

} // namespace hf
