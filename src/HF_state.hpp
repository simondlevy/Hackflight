/*
   Datatype declarations for vehicle state

   Copyright (c) 2018 Simon D. Levy

   MIT License
   */

#pragma once

#include "HF_filters.hpp"

namespace hf {

    typedef struct {

        float x;
        float dx;
        float y;
        float dy;
        float z;
        float dz;
        float phi;
        float dphi;
        float theta;
        float dtheta;
        float psi;
        float dpsi;

    } state_t;


    class State {

        friend class HackflightPure;
        friend class HackflightFull;

        private:

            static constexpr float MAX_ARMING_ANGLE_DEGREES = 25;

            bool safeAngle(float angle)
            {
                return fabs(angle) < Filter::deg2rad(MAX_ARMING_ANGLE_DEGREES);
            }

        public:

            bool armed = false;
            bool failsafe = false;

            state_t state;

            State(bool start_armed=false)
            {
                armed = start_armed;
            }

            bool safeToArm(void)
            {
                return safeAngle(state.phi) && safeAngle(state.theta);
            }

    }; // class State

} // namespace hf
