/*
   Support for sensor-fusion IMUs

   Copyright (c) 2021 Simon D. Levy

   MIT License
   */

#pragma once

#include <HF_sensor.hpp>
#include <HF_filters.hpp>

#include "../stream_imu.h"

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

    class IMU : public Sensor {

        friend class HackflightFull;

        protected:

            virtual void modifyState(State * state, uint32_t time_usec)
            {
                (void)time_usec;

                if (stream_imuGotGyrometer) {

                    // Convert degrees / sec to radians / sec
                    state->x[State::DPHI]   = radians(stream_imuGyrometerX);
                    state->x[State::DTHETA] = radians(stream_imuGyrometerY);
                    state->x[State::DPSI]   = radians(stream_imuGyrometerZ);
                }

                if (stream_imuGotQuaternion) {

                    Filter::quat2euler(
                            stream_imuQuaternionW,
                            stream_imuQuaternionX,
                            stream_imuQuaternionY,
                            stream_imuQuaternionZ, 
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

    };  // class IMU

} // namespace hf
