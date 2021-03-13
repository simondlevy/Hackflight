/*
   State subclass for miniature aerial vehicles

   Copyright (c) 2021 D. Levy

   MIT License
 */

#pragma once

#include <RFT_filters.hpp>
#include <RFT_state.hpp>

namespace hf {

    class MavState : public rft::State {

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

            float x[STATE_SIZE];

        private:

            bool safeAngle(uint8_t axis)
            {
                return fabs(x[STATE_PHI+2*axis]) < rft::Filter::deg2rad(MAX_ARMING_ANGLE_DEGREES);
            }

        public:

            virtual bool safeToArm(void) 
            {
                return safeAngle(AXIS_ROLL) && safeAngle(AXIS_PITCH);
            }

    }; // class MavState

} // namespace hf
