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

            static constexpr float KP_YAW = 0.3;           
            static constexpr float KI_YAW = 0.05;          
            static constexpr float KD_YAW = 0.00015;       

        public:

            static void run(
                    const float dt,
                    const bool reset,
                    const hf::vehicleState_t & state,
                    const hf::demands_t & demands_in,
                    hf::demands_t & demands_out)
            {
                static float integral_roll, integral_roll_prev, derivative_roll;
                static float integral_pitch, integral_pitch_prev, derivative_pitch;
                static float error_YAW_prev, integral_YAW, integral_YAW_prev, derivative_YAW;

                const float error_roll = demands_in.roll - state.phi;
                integral_roll = integral_roll_prev + error_roll*dt;
                if (reset) {   
                    integral_roll = 0;
                }
                integral_roll = constrain(integral_roll, -I_LIMIT, I_LIMIT); 

                derivative_roll = state.dphi;
                demands_out.roll = 0.01*(KP_PITCH_ROLL*error_roll + KI_PITCH_ROLL*integral_roll
                        - KD_PITCH_ROLL*derivative_roll); 

                const float error_pitch = demands_in.pitch - state.theta;
                integral_pitch = integral_pitch_prev + error_pitch*dt;
                if (reset) {   

                    integral_pitch = 0;
                }
                integral_pitch = constrain(integral_pitch, -I_LIMIT, I_LIMIT); 

                derivative_pitch = state.dtheta;
                demands_out.pitch = .01*(
                        KP_PITCH_ROLL*error_pitch +
                        KI_PITCH_ROLL*integral_pitch -
                        KD_PITCH_ROLL*derivative_pitch); 

                const float error_YAW = demands_in.yaw - state.dpsi;
                integral_YAW = integral_YAW_prev + error_YAW*dt;
                if (reset) {   
                    integral_YAW = 0;
                }
                integral_YAW = constrain(integral_YAW, -I_LIMIT, I_LIMIT); 

                derivative_YAW = (error_YAW - error_YAW_prev)/dt; 
                demands_out.yaw = .01*(KP_YAW*error_YAW + KI_YAW*integral_YAW + KD_YAW*derivative_YAW); 

                integral_roll_prev = integral_roll;

                integral_pitch_prev = integral_pitch;

                error_YAW_prev = error_YAW;
                integral_YAW_prev = integral_YAW;
            }
    };

}
