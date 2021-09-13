/*
   Support for Pesky Products Unltimate Sensor Fusion Solution IMU

   Copyright (c) 2021 Simon D. Levy

   MIT License
   */

#pragma once

#include "../sensor.hpp"
#include "../filters.hpp"
#include "../../copilot.h"

#include <USFS_Master.h>

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

    class USFS : public hf::Sensor {

        friend class Hackflight;

        private:

            USFS_Master _usfs;

        protected:

            void begin(void)
            {
            }

            virtual void modifyState(hf::State * state, float time)
            {
                State * hfstate = (State *)state;

                (void)time;

                // Convert degrees / sec to radians / sec
                hfstate->x[State::DPHI] = radians(copilot_gyrometerX);
                hfstate->x[State::DTHETA] = radians(copilot_gyrometerY);
                hfstate->x[State::DPSI] = radians(copilot_gyrometerZ);

                hf::Filter::quat2euler(
                        copilot_quaternionW,
                        copilot_quaternionX,
                        copilot_quaternionY,
                        copilot_quaternionZ, 
                        hfstate->x[State::PHI],
                        hfstate->x[State::THETA],
                        hfstate->x[State::PSI]);

                // Adjust rotation so that nose-up is positive
                hfstate->x[State::THETA] = -hfstate->x[State::THETA];

                // Convert heading from [-pi,+pi] to [0,2*pi]
                if (hfstate->x[State::PSI] < 0) {
                    hfstate->x[State::PSI] += 2*M_PI;
                }

            } // modifyState

    };  // class USFS

} // namespace hf
