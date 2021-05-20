/*
   Hackflight IMU class implementation for MulticopterSim

   Copyright(C) 2020 Simon D.Levy

   MIT License
   */

#pragma once

#include <imu.hpp>

class SimIMU : public hf::IMU {

    private:
   
		static const uint8_t Q_DIVISOR = 5; // this many gyro readings per quaternion reading

        double _quat[4] = {0};
        double _gyro[3] = {0};

        uint8_t _qcount = 0;

    public:

        virtual bool getGyrometer(float & gx, float & gy, float & gz) override
        {
            gx = _gyro[0];
            gy = _gyro[1];
            gz = _gyro[2];

            return true;
        }

        virtual bool getQuaternion(float & qw, float & qx, float & qy, float & qz, float time) override
        {
            (void)time;	

            _qcount = ((_qcount+1) % Q_DIVISOR);

            if (_qcount) return false;

            qw =   _quat[0];
            qx = - _quat[1];// invert X
            qy = - _quat[2];// invert Y
            qz =   _quat[3];

            return true;
        }

        void set(const double quat[4], const double gyro[3])
        {
            // Copy in quaternion
            for (uint8_t j=0; j<4; ++j) {
                _quat[j] = quat[j];
            }

            // Copy in gyro
            for (uint8_t j=0; j<3; ++j) {
                _gyro[j] = gyro[j];
            }
        }

}; // class SimIMU
