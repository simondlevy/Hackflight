/*
   Support for treating quaternion as a sensor
   
   Supports IMUs like EM7180 SENtral Sensor Fusion solution, where 
   quaternion is computed in hardware, and simulation platforms like
   UE4, where quaternion is provided by physics engine. For other IMUs 
   and simulators, you can use quaternion-filter classes in filters.hpp.

   Copyright (c) 2018 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <math.h>

#include "sensors/surfacemount.hpp"

namespace hf {

    class Quaternion : public SurfaceMountSensor {

        friend class Hackflight;

        private:

            float _w = 0;
            float _x = 0;
            float _y = 0;
            float _z = 0;

        protected:

            Quaternion(void)
            {
                _w = 0;
                _x = 0;
                _y = 0;
                _z = 0;
            }

            virtual void modifyState(state_t & state, float time) override
            {
                (void)time;

                float qw = _w, qx = _x, qy = _y, qz = _z;

                imu->adjustQuaternion(qw, qx, qy, qz);

                computeEulerAngles(qw, qx, qy, qz, state.rotation);
                computeEulerAngles(qw, qx, qy, qz, state.x[STATE_PHI], state.x[STATE_THETA], state.x[STATE_PSI]);

                // Adjust rotation so that nose-up is positive
                state.rotation[1] = -state.rotation[1];
                state.x[STATE_THETA] = -state.x[STATE_THETA];

                imu->adjustEulerAngles(state.rotation[0], state.rotation[1], state.rotation[2]);
                imu->adjustEulerAngles(state.x[STATE_PHI], state.x[STATE_THETA], state.x[STATE_PSI]);

                Debugger::printf("%+3.3f  %+3.3f", state.rotation[1], state.x[STATE_THETA]);

                // Convert heading from [-pi,+pi] to [0,2*pi]
                if (state.rotation[2] < 0) {
                    state.rotation[2] += 2*M_PI;
                    state.x[STATE_PSI] += 2*M_PI;
                }
            }

            virtual bool ready(float time) override
            {
                return imu->getQuaternion(_w, _x, _y, _z, time);
            }

        public:

            // We make this public so we can use it in different sketches

            static void computeEulerAngles(float qw, float qx, float qy, float qz, float euler[3])
            {
                euler[0] = atan2(2.0f*(qw*qx+qy*qz), qw*qw-qx*qx-qy*qy+qz*qz);
                euler[1] = asin(2.0f*(qx*qz-qw*qy));
                euler[2] = atan2(2.0f*(qx*qy+qw*qz), qw*qw+qx*qx-qy*qy-qz*qz);
            }

            static void computeEulerAngles(float qw, float qx, float qy, float qz,
                                           float & ex, float & ey, float & ez)
            {
                ex = atan2(2.0f*(qw*qx+qy*qz), qw*qw-qx*qx-qy*qy+qz*qz);
                ey = asin(2.0f*(qx*qz-qw*qy));
                ez = atan2(2.0f*(qx*qy+qw*qz), qw*qw+qx*qx-qy*qy-qz*qz);
            }

    };  // class Quaternion

} // namespace hf
