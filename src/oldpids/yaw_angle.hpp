/*
  Yaw angle PID control algorithm for real and simulated flight controllers
 
  Copyright (C) 2024 Simon D. Levy
 
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, in version 3.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

/*
cap angle = angle''

  where angle' = if angle > 180 then angle - 360 else angle

        angle'' = if angle' < (-180) then angle' + 360 else angle'
*/


/*
Demand is input as desired angle normalized to [-1,+1] and output
as degrees per second, both nose-right positive.
*/

#include <utils.hpp>

namespace hf {

    class YawAnglePid {

        private:

            static constexpr float KP = 6;
            static constexpr float KI = 1;
            static constexpr float KD = 0.25;
            static constexpr float ILIMIT = 360;
            //static constexpr float ANGLE_MAX = 200;

            float _integral;
            float _error;

        public:

            float run(const float dt, const float demand, const float angle)
            {
                const auto error = demand - angle;

                _integral = Utils::fconstrain(_integral + error * dt, ILIMIT);

                const auto output =
                    KP * error + KI * _integral + KD * (error - _error) / dt;

                _error = error;

                return output;
            }
    };
}
