/*
   Support for Pesky Products Unltimate Sensor Fusion Solution IMU

   Copyright (c) 2021 Simon D. Levy

   MIT License
   */

#pragma once

#include <HF_sensor.hpp>
#include <HF_filters.hpp>

extern bool  copilot_usfsGotGyrometer;
extern bool  copilot_usfsGotQuaternion;
extern float copilot_usfsGyrometerX;
extern float copilot_usfsGyrometerY;
extern float copilot_usfsGyrometerZ;
extern float copilot_usfsQuaternionW;
extern float copilot_usfsQuaternionX;
extern float copilot_usfsQuaternionY;
extern float copilot_usfsQuaternionZ;

namespace hf {

    /*
       The most common aeronautical convention defines roll as acting about
       the longitudinal axis, positive with the starboard (right) wing
       down. Yaw is about the vertical body axis, positive with the nose to
       starboard. Pitch is about an axis perpendicular to the longitudinal
       plane of symmetry, positive nose up.

        https://en.wikipedia.org/wiki/Flight_dynamics_(fixed-wing_aircraft)

        https://emissarydrones.com/what-is-roll-pitch-and-yaw
    */

    class USFS : public Sensor {

        friend class HackflightFull;

        protected:

            virtual void modifyState(State * state, uint32_t time_usec)
            {
                (void)time_usec;

                if (copilot_usfsGotGyrometer) {

                    // Convert degrees / sec to radians / sec
                    state->x[State::DPHI]   = radians(copilot_usfsGyrometerX);
                    state->x[State::DTHETA] = radians(copilot_usfsGyrometerY);
                    state->x[State::DPSI]   = radians(copilot_usfsGyrometerZ);
                }

                if (copilot_usfsGotQuaternion) {

                    Filter::quat2euler(
                            copilot_usfsQuaternionW,
                            copilot_usfsQuaternionX,
                            copilot_usfsQuaternionY,
                            copilot_usfsQuaternionZ, 
                            state->x[State::PHI],
                            state->x[State::THETA],
                            state->x[State::PSI]);

                    // Adjust rotation so that nose-up is positive
                    state->x[State::THETA] = -state->x[State::THETA];

                    // Convert heading from [-pi,+pi] to [0,2*pi]
                    if (state->x[State::PSI] < 0) {
                        state->x[State::PSI] += 2*M_PI;
                    }
                }

            } // modifyState

    };  // class USFS

} // namespace hf
