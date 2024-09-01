/*
  Yaw rate PID control algorithm for real and simulated flight controllers
 
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


namespace hf {

    class YawRatePid {

        private:

            static constexpr float KP = 120;
            static constexpr float KI = 16.7;
            static constexpr float ILIMIT = 166.7;

            float _integral;

        public:

                float run(const float dt, const float demand, const float dangle)
                {
                    const auto error = demand - dangle;

                    _integral = Utils::fconstrain(_integral + error * dt, ILIMIT);

                    const auto output = KP * error + KI * _integral;

                    return output;
                }
    };

}
