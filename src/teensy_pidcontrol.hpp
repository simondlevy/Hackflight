/*
   PID controllers for Teensy-based Hackflight

   Based on  https://github.com/nickrehm/dRehmFlight

   Copyright (C) 2026 Simon D. Levy

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

#include <hackflight.h>
#include <datatypes.h>

namespace hf {

    class PidControl {

        private:

            static constexpr float I_LIMIT = 25.0;     

            static constexpr float KP_PITCH_ROLL = 0.2;    
            static constexpr float KI_PITCH_ROLL = 0.3;    
            static constexpr float KD_PITCH_ROLL = 0.05;   

            static constexpr float KP_yaw = 0.3;           
            static constexpr float KI_yaw = 0.05;          
            static constexpr float KD_yaw = 0.00015;       

            static float runPitchRoll(
                    const float dt,
                    const bool reset,
                    const float demand,
                    const float angle,
                    const float dangle,
                    float & integral_prev)
            {
                const float error = demand - angle;

                const float integral = reset ? 0 :
                    constrain(integral_prev + error * dt, -I_LIMIT, +I_LIMIT); 

                integral_prev = integral;

                return 0.01*(
                        KP_PITCH_ROLL * error +
                        KI_PITCH_ROLL * integral -
                        KD_PITCH_ROLL * dangle); 
            }

        public:

            static void run(
                    const float dt,
                    const bool reset,
                    const hf::vehicleState_t & state,
                    const hf::demands_t & demands_in,
                    hf::demands_t & demands_out)
            {
                static float integral_roll_prev;

                demands_out.roll = runPitchRoll(dt, reset, demands_in.roll,
                        state.phi, state.dphi, integral_roll_prev);

                static float integral_pitch_prev;

                demands_out.pitch = runPitchRoll(dt, reset, demands_in.pitch,
                        state.theta, state.dtheta, integral_pitch_prev);

                static float integral_yaw_prev;

                const float error_yaw = demands_in.yaw - state.dpsi;

                const float integral_yaw = reset ? 0 :
                    constrain(integral_yaw_prev + error_yaw * dt, -I_LIMIT, +I_LIMIT); 

                integral_yaw_prev = integral_yaw;

                static float error_yaw_prev;

                const float derivative_yaw = (error_yaw - error_yaw_prev) / dt; 

                error_yaw_prev = error_yaw;

                demands_out.yaw = .01*(KP_yaw*error_yaw + KI_yaw*integral_yaw +
                        KD_yaw*derivative_yaw); 
            }
    };

}
