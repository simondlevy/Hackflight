/*
 * Dynamics class for quad-X frames using ArduPilot motor layout:
 *
 *    3cw   1ccw
 *       \ /
 *        ^
 *       / \
 *    2ccw  4cw
 *
 * Copyright (C) 2019 Simon D. Levy, Daniel Katzav
 *
 * MIT License
 */

#pragma once

#include "../FixedPitch.hpp"

class QuadXAPDynamics : public FixedPitchDynamics {

    public:	

        QuadXAPDynamics(Dynamics::vehicle_params_t &vparams, FixedPitchDynamics::fixed_pitch_params_t &fparams)
            : FixedPitchDynamics(4, vparams, fparams)
        {
        }

    protected:

        virtual int8_t getRotorDirection(uint8_t i) override
        {
            static const int8_t d[4] = {-1, -1, +1, +1};
            return d[i];
        }

        virtual int8_t getRotorRollContribution(uint8_t i) override
        {
            static const int8_t r[4] = {-1, +1, +1, -1};
            return r[i];
        }

        virtual int8_t getRotorPitchContribution(uint8_t i) override
        {
            static const int8_t p[4] = {-1, +1, -1, +1};
            return p[i];
        }

}; // class QuadXAP
