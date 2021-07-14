/*
   Datatype declarations for vehicle state

   Copyright (c) 2018 Simon D. Levy

   MIT License
   */

#pragma once

#include <RFT_state.hpp>
#include <RFT_filters.hpp>

namespace hf {

    class State : public rft::State {

        friend class Hackflight;

        private:

            static constexpr float MAX_ARMING_ANGLE_DEGREES = 25;

            bool safeAngle(uint8_t axis)
            {
                return fabs(x[axis]) < rft::Filter::deg2rad(MAX_ARMING_ANGLE_DEGREES);
            }

        protected:

            bool safeToArm(void)
            {
                return safeAngle(PHI) && safeAngle(THETA);
            }


        public:

            // See Bouabdallah et al. (2004)
            enum {X, DX, Y, DY, Z, DZ, PHI, DPHI, THETA, DTHETA, PSI, DPSI, SIZE};

            float x[SIZE];

            State(bool start_armed=false)
                : rft::State(start_armed)
            {
            }


    }; // class State

} // namespace hf
