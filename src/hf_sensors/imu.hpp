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

            virtual void modifyState(state_t & state, uint32_t time_usec)
            {
                (void)time_usec;

                // Convert degrees / sec to radians / sec
                state.dphi   = stream_imuGotGyrometer ? radians(stream_imuGyrometerX) : state.dphi;
                state.dtheta = stream_imuGotGyrometer ? radians(stream_imuGyrometerY) : state.dtheta;
                state.dpsi   = stream_imuGotGyrometer ? radians(stream_imuGyrometerZ) : state.dpsi;

                // Convert quaternion to Euler angles
                float phi = 0;
                float theta = 0;
                float psi = 0;
                Filter::quat2euler(
                        stream_imuQuaternionW,
                        stream_imuQuaternionX,
                        stream_imuQuaternionY,
                        stream_imuQuaternionZ, 
                        phi,
                        theta,
                        psi);


                state.phi = stream_imuGotQuaternion ? phi : state.phi;

                // Adjust rotation so that nose-up is positive
                state.theta = stream_imuGotQuaternion ? -theta : state.theta;

                // Convert heading from [-pi,+pi] to [0,2*pi]
                state.psi = stream_imuGotQuaternion ? (psi < 0 ? psi + 2*M_PI : psi) : state.psi;

            } // modifyState

    };  // class IMU

} // namespace hf
