/*
   Hackflight Board class implementation for MulticopterSim

   Copyright(C) 2019 Simon D.Levy
   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MEReceiverHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
   */

#pragma once

#include <board.hpp>
#include <debugger.hpp>

class SimBoard : public hf::Board {

    private:

        float _currentTime = 0; // must be float for Hackflight

        double _quat[4] = {0};
        double _gyro[3] = {0};
        double _motors[4] = {0};

    protected:


        bool getQuaternion(float & qw, float & qx, float & qy, float & qz)
        {
            qw =   _quat[0];
            qx = - _quat[1];// invert X
            qy = - _quat[2];// invert Y
            qz =   _quat[3];

            return true;
        }

        bool getGyrometer(float & gx, float & gy, float & gz)
        {
            gx = _gyro[0];
            gy = _gyro[1];
            gz = _gyro[2];

            return true;
        }

        void writeMotor(uint8_t index, float value)
        {
            _motors[index] = value;
        }

        float getTime(void)
        {
            return _currentTime;
        }

    public:

        SimBoard() { }

        virtual ~SimBoard() { }

        void update(double time, double quat[4], double gyro[3], double * motors)
        {
            _currentTime = time;

            // Copy in quaternion
            for (uint8_t j=0; j<4; ++j) {
                _quat[j] = quat[j];
            }

            // Copy in gyro
            for (uint8_t j=0; j<3; ++j) {
                _gyro[j] = gyro[j];
            }

            // Copy out motors
            for (uint8_t j=0; j<4; ++j) {
                motors[j] = _motors[j];
            }
        }

}; // class Simboard
