/*
   Support for sensor-fusion IMUs

   Copyright (c) 2021 Simon D. Levy

   MIT License
   */

#pragma once

#include <HF_sensor.hpp>
#include <HF_utils.hpp>

#include "../copilot.h"

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

            } // modifyState

    };  // class IMU

} // namespace hf
