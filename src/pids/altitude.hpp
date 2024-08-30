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

            static constexpr float KP_Z = 2.0;
            static constexpr float KI_Z = 0.5;

            static constexpr float KP_DZ = 25;
            static constexpr float KI_DZ = 15;

            static constexpr float ILIMIT = 5000;

            float run_pi(const float dt, const float kp, const float ki,
                    const float target, const float actual, float & integral)
            {
                const auto error = target - actual;

                integral = Utils::fconstrain(integral + dt * error, ILIMIT);

                return  kp * error + ki * integral;
            }

            class ClimbRatePid {

                private:

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

                        return  KP_DZ * error + KI_DZ * _integral;
                    }
            };

            float _integral;

            ClimbRatePid _climbRatePid;

        public:

            float run(
                    const float dt,
                    const float z_target,
                    const float z_actual,
                    const float dz_actual)
            {
                const auto dz_target = run_pi(dt, KP_Z, KI_Z, z_target, z_actual, _integral);

                //const auto error = z_target - z_actual;
                //_integral = Utils::fconstrain(_integral + dt * error, ILIMIT);
                //const auto dz_target =  KP_Z * error + KI_Z * _integral;

                return _climbRatePid.run(dt, dz_target, dz_actual);
            }
    };

}
