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

        private:

            static constexpr float I_LIMIT = 25.0;     

            static constexpr float KP = 0.002;    
            static constexpr float KI = 0.003;    
            static constexpr float KD = 0.0005;   

            static constexpr float THROTTLE_DOWN = 0.06;

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

                        _integral += error * dt;

                        if (reset) {
                            _integral = 0;
                        }

                        _integral = hf::Utils::fconstrain(_integral, I_LIMIT);

                        const auto output =
                            KP * error + KI * _integral - KD * dangle; 

                        return output;
                    }

                private:

                    float _integral;
            };

            PitchRollPidController _rollPid;

            PitchRollPidController _pitchPid;

        public:

            void run(
                    const float dt, 
                    const float thro_demand, 
                    const float roll_demand, 
                    const float pitch_demand, 
                    const float phi,
                    const float theta,
                    const float dphi,
                    const float dtheta,
                    float & roll_out,
                    float & pitch_out)
            {
                const auto reset = thro_demand < THROTTLE_DOWN;

                roll_out = _rollPid.run(dt, reset, roll_demand, phi, dphi);

                pitch_out = _pitchPid.run(dt, reset, pitch_demand, theta, dtheta);
            }    

    };
}
