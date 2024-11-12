/*
 * Dynamics class for quad-X frames using Betaflight motor layout:
 *
 *    4cw   2ccw
 *       \ /
 *        ^
 *       / \
 *    3ccw  1cw
 *
 * Copyright (C) 2022 Simon D. Levy
 *
 * MIT License
 */

#pragma once

#include "../FixedPitch.hpp"

class QuadXBFDynamics : public FixedPitchDynamics {

    protected:

        virtual int8_t getRotorDirection(uint8_t i) override
        {
            static const int8_t d[4] = {-1, +1, +1, -1};
            return d[i];
        }

        int8_t getRotorRollContribution(uint8_t i)
        {
            static const int8_t d[4] = {-1, -1, +1, +1};
            return d[i];
        }

        int8_t getRotorPitchContribution(uint8_t i)
        {
            static const int8_t d[4] = {+1, -1, +1, -1};
            return d[i];
        }

    public:	

        QuadXBFDynamics(
                Dynamics::vehicle_params_t &vparams,
                FixedPitchDynamics::fixed_pitch_params_t &fparams,
                bool autoland=true)
            : FixedPitchDynamics(4, vparams, fparams, autoland)
        {
        }

}; // class QuadXBF
