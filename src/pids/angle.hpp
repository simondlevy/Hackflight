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
         * states roll_demand, pitch_demand, and yaw_demand computed in getDesState().
         * Error is simply the desired state minus the actual state (ex.
         * roll_demand - phi). Two safety features are implimented here
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

            static constexpr float KP_PITCH_ROLL = 0.002;    
            static constexpr float KI_PITCH_ROLL = 0.003;    
            static constexpr float KD_PITCH_ROLL = 0.0005;   

            static constexpr float KP_YAW = 0.003;           
            static constexpr float KI_YAW = 0.0005;          
            static constexpr float KD_YAW = 0.0000015;       

            static const uint32_t THROTTLE_DOWN = 1060;

            class PitchRollPidController {

                public:

                    float run(
                            const float dt,
                            const bool reset,
                            const float demand,
                            const float angle,
                            const float dangle)
                    {
                        const auto error = demand - angle;

                        _integral = _integral_prev + error * dt;

                        if (reset) {
                            _integral = 0;
                        }

                        _integral = hf::Utils::fconstrain(_integral, -I_LIMIT, I_LIMIT); 

                        const auto output = KP_PITCH_ROLL * error +
                                KI_PITCH_ROLL * _integral - KD_PITCH_ROLL * dangle; 

                        _integral_prev = _integral;

                        return output;
                    }

                private:

                    float _integral;
                    float _integral_prev;
            };

            class YawPidController {

                public:

                    float run(
                            const float dt,
                            const bool reset,
                            const float demand,
                            const float dangle)
                    {
                        const auto error = demand - dangle;

                        _integral = _integral_prev + error * dt;

                        if (reset) {
                            _integral = 0;
                        }

                        _integral = hf::Utils::fconstrain(_integral, -I_LIMIT, I_LIMIT); 

                        const auto derivative = (error - _error_prev) / dt;

                        const auto output =
                            KP_YAW * error + KI_YAW * _integral - KD_YAW * derivative; 

                        _integral_prev = _integral;
                        _error_prev = error;

                        return output;
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
                    const uint32_t throttle,
                    const float roll_demand, 
                    const float pitch_demand, 
                    const float yaw_demand, 
                    const float phi,
                    const float theta,
                    const float dphi,
                    const float dtheta,
                    const float dpsi,
                    float & roll_out,
                    float & pitch_out,
                    float & yaw_out) 
            {
                const auto reset = throttle < THROTTLE_DOWN;

                roll_out = _rollPid.run(dt, reset, roll_demand, phi, dphi);

                pitch_out = _pitchPid.run(dt, reset, pitch_demand, theta, dtheta);

                yaw_out = _yawPid.run(dt, reset, yaw_demand, dpsi);
            }    

    };
}
