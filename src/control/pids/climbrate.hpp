#pragma once

#include <num.hpp>
#include <vehicles/diyquad.hpp>

class ClimbRateController {

    public:

        /**
         * Demand is input as altitude target in meters and output as 
         * arbitrary positive value to be scaled according to motor
         * characteristics.
         */
        static float run(
                const bool hovering,
                const float z0,
                const float dt,
                const float z,
                const float dz,
                const float demand)
        {
            static float _integral;

            const auto airborne = hovering || (z > z0);

            const auto error = demand - dz;

            _integral = airborne ? 
                Num::fconstrain(_integral + error * dt, ILIMIT) : 0;

            const auto thrust = KP * error + KI * _integral;

            return airborne ?
                Num::fconstrain(thrust * THRUST_SCALE + THRUST_BASE,
                        THRUST_MIN, THRUST_MAX) : 0;
        }

    private:

        static constexpr float KP = 25;
        static constexpr float KI = 15;
        static constexpr float ILIMIT = 5000;
};
