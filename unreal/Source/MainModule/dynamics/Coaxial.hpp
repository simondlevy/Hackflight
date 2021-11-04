/*
 * Dynamics class for coaxial copters
 *
 * Degrees of Freedom are:
 *
 *   - Head speed 1
 *   - Head speed 2
 *   - Collective (blade) pitch
 *   - Cyclic pitch
 *   - Cyclic roll
 * 
 * Copyright (C) 2021 Simon D. Levy
 *
 * MIT License
 */

#pragma once

#include "../Dynamics.hpp"

class CoaxialDynamics : public Dynamics {

    private:

        
        static constexpr double FAKE_COLLECTIVE = 5.E-06; // XXX

        static constexpr double CYCLIC_COEFFICIENT = 1.0;

        static double computeCyclic(double * actuators, uint8_t axis)
        {
            return CYCLIC_COEFFICIENT * actuators[axis];
        }

    protected:

        virtual double getThrustCoefficient(double * actuators) override
        {
            (void)actuators;
            return FAKE_COLLECTIVE;
        }

        virtual void computeRollAndPitch(double * actuators, double * omegas2, double & roll, double & pitch) override
        {
            // For a coaxial, rotor speeds do not determine roll and pitch
            (void)omegas2;
            
            roll = computeCyclic(actuators, 3);
            pitch = computeCyclic(actuators, 4);
         }

        // motor direction for animation
        virtual int8_t getRotorDirection(uint8_t i) override
        {
            const int8_t dir[2] = {-1, +1};
            return dir[i];
        }

    public:	

        CoaxialDynamics(Dynamics::vehicle_params_t &vparams)
            : Dynamics(5, vparams)
        {
            _rotorCount = 2;
        }

}; // class CoaxialDynamics
