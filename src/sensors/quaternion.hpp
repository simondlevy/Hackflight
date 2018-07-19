/*
   quaternion.hpp : Support for treating quaternion as a sensor
   
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

#include <cmath>
#include <math.h>

#include "debug.hpp"
#include "sensor.hpp"
#include "board.hpp"

namespace hf {

    class Quaternion : public Sensor {

        friend class Hackflight;

        protected:

            void init(Board * board) 
            {
                _board = board;

                memset(_quat, 0, 4*sizeof(float));
            }

            virtual void modifyState(State & state, float time) override
            {
                (void)time;

                float * q = _quat; // abbreviate

                state.eulerAngles[0] = atan2(2.0f*(q[0]*q[1]+q[2]*q[3]),q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3]);
                state.eulerAngles[1] =  asin(2.0f*(q[1]*q[3]-q[0]*q[2]));
                state.eulerAngles[2] = atan2(2.0f*(q[1]*q[2]+q[0]*q[3]),q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3]);

                // Convert heading from [-pi,+pi] to [0,2*pi]
                if (state.eulerAngles[2] < 0) {
                    state.eulerAngles[2] += 2*M_PI;
                }
            }

            virtual bool ready(float time) override
            {
                (void)time;

                if (_board->getQuaternion(_quat)) {
                    return true;
                }

                return false;
            }

        private:

            // Gyro is a surface-mount sensor
            Board * _board;

            float _quat[4];

    };  // class Quaternion

} // namespace
