/*
   Support for gyrometer (a.k.a. gyroscope) 

   Copyright (c) 2018 Simon D. Levy

   MIT License
 */

#pragma once

#include <math.h>

#include "sensors/surfacemount.hpp"

namespace hf {

    class Gyrometer : public SurfaceMountSensor {

        friend class Hackflight;

        private:

            float _x = 0;
            float _y = 0;
            float _z = 0;

        protected:

            virtual void modifyState(state_t & state, float time) override
            {
                (void)time;

                // Compensate for IMU mounting as needed
                imu->adjustGyrometer(_x, _y, _z);

                // NB: We negate gyro X, Y to simplify PID controller
                state.angularVel[0] =  _x;
                state.angularVel[1] = -_y;
                state.angularVel[2] = -_z;
            }

            virtual bool ready(float time) override
            {
                (void)time;

                bool result = imu->getGyrometer(_x, _y, _z);

                return result;
            }

        public:

            Gyrometer(void)
            {
                _x = 0;
                _y = 0;
                _z = 0;
            }

    };  // class Gyrometer

} // namespace
