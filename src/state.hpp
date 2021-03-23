/*
   State subclass for miniature aerial vehicles

   Copyright (c) 2021 D. Levy

   MIT License
 */

#pragma once

#include <RFT_filters.hpp>
#include <RFT_state.hpp>

namespace hf {

    class State : public rft::State {

        private:

            static constexpr float MAX_ARMING_ANGLE_DEGREES = 25.0f;

        public:

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
                STATE_Z,            // NED, so + downward
                STATE_DZ,
                STATE_PHI,          // + roll right
                STATE_DPHI,
                STATE_THETA,        // + nose up
                STATE_DTHETA,
                STATE_PSI,          // + yaw right
                STATE_DPSI,
                STATE_SIZE
            };

            float x[STATE_SIZE];

        private:

            bool safeAngle(uint8_t axis)
            {
                return fabs(x[STATE_PHI+2*axis]) < rft::Filter::deg2rad(MAX_ARMING_ANGLE_DEGREES);
            }

        public:

            virtual bool safeToArm(void) override
            {
                return safeAngle(AXIS_ROLL) && safeAngle(AXIS_PITCH);
            }

    }; // class State

} // namespace hf
