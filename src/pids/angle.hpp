/*
  Angle PID controller

  Adapted from https://github.com/nickrehm/dRehmFlight
 
  Copyright (C) 2024 Simon D. Levy
 
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, in version 3.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#pragma once

#include <hackflight.hpp>
#include <utils.hpp>

namespace hf {

    class AnglePid {

        //DESCRIPTION: Computes control commands based on state error (angle)
        /*
         * Basic PID control to stablize on angle setpoint based on desired
         * states roll_des, pitch_des, and yaw_des computed in getDesState().
         * Error is simply the desired state minus the actual state (ex.
         * roll_des - roll_IMU). Two safety features are implimented here
         * regarding the I terms. The I terms are saturated within specified
         * limits on startup to prevent excessive buildup. This can be seen by
         * holding the vehicle at an angle and seeing the motors ramp up on one
         * side until they've maxed out throttle...saturating I to a specified
         * limit fixes this. The second feature defaults the I terms to 0 if
         * the throttle is at the minimum setting. This means the motors will
         * not start spooling up on the ground, and the I terms will always
         * start from 0 on takeoff. This function updates the variables
         * roll_PID, pitch_PID, and yaw_PID which can be thought of as 1-D
         * stablized signals. They are mixed to the configuration of the
         * vehicle in the mixer.
         */

        private:

            static constexpr float I_LIMIT = 25.0;     

            static constexpr float KP_PITCH_ROLL = 0.2;    
            static constexpr float KI_PITCH_ROLL = 0.3;    
            static constexpr float KD_PITCH_ROLL = 0.05;   

            static constexpr float KP_YAW = 0.3;           
            static constexpr float KI_YAW = 0.05;          
            static constexpr float KD_YAW = 0.00015;       

            static constexpr float SCALE = 0.01;

            static const uint32_t THROTTLE_DOWN = 1060;

            class PitchRollPidController {

                public:

                    void run(
                            const float dt,
                            const float des,
                            const float imu,
                            const bool reset,
                            const float gyro,
                            float & pid)
                    {
                        const auto error = des - imu;

                        _integral = _integral_prev + error * dt;

                        if (reset) {
                            _integral = 0;
                        }

                        _integral = hf::Utils::fconstrain(_integral, -I_LIMIT, I_LIMIT); 

                        const auto derivative = gyro;

                        pid = SCALE * (KP_PITCH_ROLL * error +
                                KI_PITCH_ROLL * _integral - KD_PITCH_ROLL * derivative); 

                        _integral_prev = _integral;
                    }


                private:

                    float _integral;
                    float _integral_prev;
            };

            class YawPidController {

                public:

                    void run(
                            const float dt,
                            const float des,
                            const bool reset,
                            const float gyro,
                            float & pid)
                    {
                        const auto error = des - gyro;

                        _integral = _integral_prev + error * dt;

                        if (reset) {
                            _integral = 0;
                        }

                        _integral = hf::Utils::fconstrain(_integral, -I_LIMIT, I_LIMIT); 

                        const auto derivative = (error - _error_prev) / dt;

                        pid = SCALE * (KP_YAW * error + KI_YAW * _integral - KD_YAW * derivative); 

                        _integral_prev = _integral;
                        _error_prev = error;
                    }


                private:

                    float _integral;
                    float _integral_prev;
                    float _error_prev;
            };

            PitchRollPidController _rollPid;

            PitchRollPidController _pitchPid;

            YawPidController _yawPid;

        public:

            void run(
                    const float dt, 
                    const float roll_des, 
                    const float pitch_des, 
                    const float yaw_des, 
                    const float roll_IMU,
                    const float pitch_IMU,
                    const uint32_t throttle,
                    const float GyroX,
                    const float GyroY,
                    const float GyroZ,
                    float & roll_PID,
                    float & pitch_PID,
                    float & yaw_PID) 
            {
                const auto reset = throttle < THROTTLE_DOWN;

                _rollPid.run(dt, roll_des, roll_IMU, reset, GyroX, roll_PID);

                _pitchPid.run(dt, pitch_des, pitch_IMU, reset, GyroY, pitch_PID);

                _yawPid.run(dt, yaw_des, reset, GyroZ, yaw_PID);
            }    

    };
}
