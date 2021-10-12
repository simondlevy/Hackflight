/*
   Safety features for Hackflight

   Copyright (c) 2018 Simon D. Levy

   MIT License
   */

#pragma once

#include "HF_state.hpp"
#include "HF_filters.hpp"

namespace hf {

    class Safety {

        friend class HackflightFull;

        private:

            static constexpr float MAX_ARMING_ANGLE_DEGREES = 25;

            static bool safeAngle(float angle)
            {
                return fabs(angle) < Filter::deg2rad(MAX_ARMING_ANGLE_DEGREES);
            }

        public:

            bool armed = false;
            bool failsafe = false;

            static bool safeToArm(state_t & state)
            {
                return safeAngle(state.phi) && safeAngle(state.theta);
            }

    }; // class Saftey

} // namespace hf
