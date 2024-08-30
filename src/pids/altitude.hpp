/*
   Altitude PID controller

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

namespace hf {

    class AltitudePid {

        private:

            class ClimbRatePid {

                private:

                    static constexpr float KP = 25;
                    static constexpr float KI = 15;
                    static constexpr float ILIMIT = 5000;

                    float _integral;

                public:

                    float run(
                            const float dt, 
                            float dz_target, 
                            const float dz_actual)
                    {
                        const auto error = dz_target - dz_actual;

                        _integral =
                            Utils::fconstrain(_integral + dt * error, ILIMIT);

                        return  KP * error + KI * _integral;
                    }
            };

            static constexpr float KP = 2.0;
            static constexpr float KI = 0.5;
            static constexpr float ILIMIT = 5000;

            float _integral;

            ClimbRatePid _climbRatePid;

        public:

            float run(
                    const float dt,
                    const float z_target,
                    const float z_actual,
                    const float dz_actual)
            {
                const auto error = z_target - z_actual;

                _integral = Utils::fconstrain(_integral + dt * error, ILIMIT);

                const auto dz_target =  KP * error + KI * _integral;

                return _climbRatePid.run(dt, dz_target, dz_actual);
            }
    };

}
