#pragma once

#include <num.hpp>

class AltitudeController {

    public:

        /**
         * Demand is input as altitude target in meters and output as 
         * climb rate in meters per second.
         */
        static float run(
                const bool hovering,
                const float dt,
                const float z,
                const float thrust)
        {
            static float _integral;

            const auto error = thrust - z;

            _integral = hovering ?
                Num::fconstrain(_integral + error * dt, ILIMIT) : 0;

            return hovering ? 
                Num::fconstrain(KP * error + KI * _integral,
                        fmaxf(VEL_MAX, 0.5f)  * VEL_MAX_OVERHEAD) :
                -LANDING_SPEED_MPS;
        }

    private:

        static constexpr float KP = 2;
        static constexpr float KI = 0.5;
        static constexpr float ILIMIT = 5000;
        static constexpr float VEL_MAX = 1;
        static constexpr float VEL_MAX_OVERHEAD = 1.10;
        static constexpr float LANDING_SPEED_MPS = 0.15;
};
