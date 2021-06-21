/*
   Mock IMU for testing

   Copyright (c) 2020 Simon D. Levy

   MIT License
 */

#pragma once

#include "imu.hpp"

namespace hf {

    class MockIMU : public IMU {

        public:

            virtual bool getGyrometer(float & gx, float & gy, float & gz) override
            {
                gx = 0;
                gy = 0;
                gz = 0;

                return true;
            }

            virtual bool getQuaternion(float & qw, float & qx, float & qy, float & qz, float time) override
            {
                (void)time;

                qw = 0;
                qx = 0;
                qy = 0;
                qz = 0;

                return true;
            }

    }; // class MockIMU

} // namespace hf
