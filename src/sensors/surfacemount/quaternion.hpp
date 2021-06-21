/*
   Support for treating quaternion as a sensor
   
   Supports IMUs like EM7180 SENtral Sensor Fusion solution, where 
   quaternion is computed in hardware, and simulation platforms like
   UE4, where quaternion is provided by physics engine. For other IMUs 
   and simulators, you can use quaternion-filter classes in filters.hpp.

   Copyright (c) 2018 Simon D. Levy

   MIT License
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

                computeEulerAngles(_w, _x, _y, _z, state.rotation);

                // Convert heading from [-pi,+pi] to [0,2*pi]
                if (state.rotation[2] < 0) {
                    state.rotation[2] += 2*M_PI;
                }

                // Compensate for different mounting orientations
                imu->adjustEulerAngles(state.rotation[0], state.rotation[1], state.rotation[2]);

            }

            virtual bool ready(float time) override
            {
                return imu->getQuaternion(_w, _x, _y, _z, time);
            }

        public:

            // We make this public so we can use it in different sketches
            static void computeEulerAngles(float qw, float qx, float qy, float qz, float euler[3])
            {
                euler[0] =  atan2(2.0f*(qw*qx+qy*qz),qw*qw-qx*qx-qy*qy+qz*qz);
                euler[1] =   asin(2.0f*(qx*qz-qw*qy));
                euler[2] =  atan2(2.0f*(qx*qy+qw*qz),qw*qw+qx*qx-qy*qy-qz*qz);
            }

    };  // class Quaternion

} // namespace hf
