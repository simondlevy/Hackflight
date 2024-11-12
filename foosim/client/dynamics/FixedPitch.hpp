/*
 * Header-only code for dynamics of vehicles with fixed rotor pitch
 * (quadcopter, hexacopter, ocotocopter, ...)
 *
 * Copyright (C) 2021 Simon D. Levy
 *
 * MIT License
 */

#pragma once

#include "../Dynamics.hpp"

class FixedPitchDynamics : public Dynamics {

    public:

        /**
         *  Vehicle parameters
         */
        typedef struct {

            double b;  // thrust coefficient [F=b*w^2]
            double l;  // arm length [m]

        } fixed_pitch_params_t; 

    private:

        fixed_pitch_params_t _fparams;

    protected:

        FixedPitchDynamics(
                uint8_t nmotors,
                Dynamics::vehicle_params_t &vparams,
                fixed_pitch_params_t &fparams,
                bool autoland=true)
            : Dynamics(nmotors, vparams, autoland)
        {
            memcpy(&_fparams, &fparams, sizeof(fixed_pitch_params_t));
        }


        virtual double getThrustCoefficient(double * actuators) override
        {
            // Thrust coefficient is constant for fixed-pitch rotors

            (void)actuators;
            
            return _fparams.b;
        }

        virtual void computeRollAndPitch(
                double * actuators,
                double * omegas2,
                double & roll,
                double & pitch) override
        {
            // We've already used actuators to compute omegas2
            (void)actuators;

            roll = 0;
            pitch = 0;

            for (uint8_t i=0; i<_rotorCount; ++i) {
                roll += _fparams.l * _fparams.b * omegas2[i] * getRotorRollContribution(i);
                pitch += _fparams.l * _fparams.b * omegas2[i] * getRotorPitchContribution(i);
            }
        }

        virtual int8_t getRotorDirection(uint8_t i) = 0;

        virtual int8_t getRotorRollContribution(uint8_t i) = 0;

        virtual int8_t getRotorPitchContribution(uint8_t i)= 0;

 
}; // class FixedPitchDynamics
