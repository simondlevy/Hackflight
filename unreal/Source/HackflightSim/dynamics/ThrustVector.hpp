/*
 * Dynamics class for thrust vectoring
 *
 * Copyright (C) 2020 Simon D. Levy, Noah Ghosh
 *
 * MIT License
 */

#pragma once

#include "../Dynamics.hpp"
#define _USE_MATH_DEFINES
#include <math.h>

class ThrustVectorDynamics : public Dynamics {

    private:

        // arbitrary for now
        static constexpr float THRUST_COEFFICIENT = 1.75E-05;

        // radians
        float _nozzleMaxAngle = 0;

        float computeNozzle(float * motorvals, float * omegas2, uint8_t axis)
        {
            return THRUST_COEFFICIENT * (omegas2[0] + omegas2[1]) * sin(motorvals[axis] * _nozzleMaxAngle);
        }

    protected:

        // Dynamics method overrides

        virtual void computeRollAndPitch(float * motorvals, float * omegas2, float & roll, float & pitch) override
        {
            roll = computeNozzle(motorvals, omegas2, 2);
            pitch = computeNozzle(motorvals, omegas2, 3);
        }

        // motor direction for animation
        virtual int8_t getRotorDirection(uint8_t i) override
        {
            const int8_t dir[2] = {-1, +1};
            return dir[i];
        }

        virtual float getThrustCoefficient(float * motorvals)
        {
            (void)motorvals;

            return THRUST_COEFFICIENT;
        }

    public:	

        ThrustVectorDynamics(Dynamics::vehicle_params_t &vparams, float nozzleMaxAngle)
            : Dynamics(4, vparams)
        {
            _rotorCount = 2;

            // degrees => radians
            _nozzleMaxAngle = M_PI * nozzleMaxAngle / 180;
        }

}; // class ThrustVectorDynamics
