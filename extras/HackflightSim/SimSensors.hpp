/*
   Simulate sensors using vehicle dynamics

   Copyright(C) 2019 Simon D.Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MEReceiverHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
   */


#pragma once

#include "dynamics/MultirotorDynamics.hpp"

#include <sensor.hpp>
#include <datatypes.hpp>
#include <debugger.hpp>

class SimSensors : public hf::Sensor {

    private:

        static void rotateVelocity(state_t & state, double eulerAngles[3], double velocityXYZ[3])
        {
            // Rotate vehicle's inertial velocity into body frame
            float psi = eulerAngles[2];
            float cp = cos(psi);
            float sp = sin(psi);
            float vx = velocityXYZ[0];
            float vy = velocityXYZ[1];
            state.velocityForward    = vx * cp + vy * sp;
            state.velocityRightward  = vy * cp - vx * sp;
        }

    protected:

        // We do all dynamcics => state conversion; subclasses just return sensor values
        MultirotorDynamics * _dynamics;

        virtual bool ready(float time) override
        {
            (void) time;
            return true;
        }

        virtual void modifyState(state_t & state, float time)
        {
            (void)time;

            // Get vehicle state from dynamics
            double angularVelocity[3] = {0};
            double earthFrameAcceleration[3] = {0};
            double eulerAngles[3] = {0};
            double velocityXYZ[3] = {0};
            double positionXYZ[3] = {0};
            _dynamics->getState(angularVelocity, earthFrameAcceleration, eulerAngles, velocityXYZ, positionXYZ);

            // Use vehicle state to modify Hackflight state values
            state.altitude   = -positionXYZ[2]; // Negate for NED => ENU conversion
            state.variometer = -velocityXYZ[2];
            rotateVelocity(state, eulerAngles, velocityXYZ);
        }

    public:

        SimSensors(MultirotorDynamics * dynamics)
        {
            _dynamics = dynamics;
        }

}; // class SimSensor
