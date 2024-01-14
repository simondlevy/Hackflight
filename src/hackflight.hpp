/**
 * Hackflight class for real and simulated flight controllers.
 *
 * Copyright (C) 2024 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <math.h>

#include <clock.hpp>
#include <datatypes.h>
#include <num.hpp>

#include <closedloops/altitude.hpp>
#include <closedloops/pitchroll_angle.hpp>
#include <closedloops/pitchroll_rate.hpp>
#include <closedloops/position.hpp>
#include <closedloops/yaw_angle.hpp>
#include <closedloops/yaw_rate.hpp>

class Hackflight {

    public:

        static const uint8_t MAX_MOTOR_COUNT = 20; // whatevs

        void init(
                const mixFun_t mixFun,
                const Clock::rate_t pidUpdateRate,
                const float thrustScale,
                const float thrustBase,
                const float thrustMin,
                const float thrustMax)
        {
            init(
                    mixFun,
                    pidUpdateRate, 
                    thrustScale, 
                    thrustBase, 
                    thrustMin, 
                    thrustMax, 
                    1, 
                    1);
         }

        void init(
                const mixFun_t mixFun,
                const Clock::rate_t pidUpdateRate,
                const float thrustScale,
                const float thrustBase,
                const float thrustMin,
                const float thrustMax,
                const float pitchRollScale,
                const float yawScale)
        {
            _mixFun = mixFun;

            _thrustScale = thrustScale;
            _thrustBase = thrustBase;
            _thrustMin = thrustMin;
            _thrustMax = thrustMax;
            _pitchRollScale = pitchRollScale;
            _yawScale = yawScale;

            initClosedLoopControllers(pidUpdateRate);
        }

        void step(
                const bool inHoverMode,
                const vehicleState_t & vehicleState,
                const demands_t & openLoopDemands,
                float motorvals[])
        {
            // Start with open-loop demands
            demands_t demands = {
                openLoopDemands.thrust,
                openLoopDemands.roll,
                openLoopDemands.pitch,
                openLoopDemands.yaw,
            };

            if (inHoverMode) {

                // In hover mode, thrust demand comes in as [-1,+1], so
                // we convert it to a target altitude in meters
                demands.thrust = Num::rescale(
                        demands.thrust, -1, +1, 0.2, 2.0);

                // Position controller converts meters per second to
                // degrees
                _positionController.run(vehicleState, demands); 

                _altitudeController.run(vehicleState, demands); 

                // Scale up thrust demand for motors
                demands.thrust = Num::fconstrain(
                        demands.thrust * _thrustScale + _thrustBase,
                        _thrustMin, _thrustMax);
            }

            else {

                // In non-hover mode, thrust demand comes in as [0,1], so we
                // scale it up for motors
                demands.thrust *= _thrustMax;

                // In non-hover mode, pitch/roll demands come in as
                // [-1,+1], which we convert to degrees for input to
                // pitch/roll controller
                demands.roll *= 30;
                demands.pitch *= 30;
            }

            _pitchRollAngleController.run(vehicleState, demands);

            _pitchRollRateController.run(vehicleState, demands);

            _yawAngleController.run(vehicleState, demands);

            _yawRateController.run(vehicleState, demands);

            // Reset closed-loop controllers on zero thrust
            if (demands.thrust == 0) {

                demands.roll = 0;
                demands.pitch = 0;
                demands.yaw = 0;

                resetControllers();
            }

            // Scale yaw, pitch and roll demands for mixer
            demands.yaw *= _yawScale;
            demands.roll *= _pitchRollScale;
            demands.pitch *= _pitchRollScale;

            // Run mixer
            runMixer(demands, motorvals);
        }

        void resetControllers(void)
        {
            _pitchRollAngleController.resetPids();
            _pitchRollRateController.resetPids();
            _positionController.resetPids();

            _altitudeController.resetFilters();
            _positionController.resetFilters();
        }

        static void gyroToVehicleState(
                const Axis3f & gyro, vehicleState_t & vehicleState)
        {
            vehicleState.dphi =    gyro.x;     
            vehicleState.dtheta = -gyro.y; // (negate for ENU)
            vehicleState.dpsi =    gyro.z; 
        }

    private:

        float _thrustScale;
        float _thrustBase;
        float _thrustMin;
        float _thrustMax;
        float _pitchRollScale;
        float _yawScale;

        mixFun_t _mixFun;

        PitchRollAngleController _pitchRollAngleController;
        PitchRollRateController _pitchRollRateController;
        PositionController _positionController;
        AltitudeController _altitudeController;
        YawAngleController _yawAngleController;
        YawRateController _yawRateController;

        void initClosedLoopControllers(const Clock::rate_t pidUpdateRate) 
        {
            _pitchRollAngleController.init(pidUpdateRate);
            _pitchRollRateController.init(pidUpdateRate);
            _yawAngleController.init(pidUpdateRate);
            _yawRateController.init(pidUpdateRate);
            _positionController.init(pidUpdateRate);
            _altitudeController.init(pidUpdateRate);
        }

        void runMixer(const demands_t & demands, float motorvals[])
        {
            const float maxAllowedThrust = UINT16_MAX;

            float uncapped[MAX_MOTOR_COUNT] = {};
            uint8_t count = 0;
            _mixFun(demands, uncapped, count);

            float highestThrustFound = 0;
            for (uint8_t k=0; k<count; k++) {

                const auto thrust = uncapped[k];

                if (thrust > highestThrustFound) {
                    highestThrustFound = thrust;
                }
            }

            float reduction = 0;
            if (highestThrustFound > maxAllowedThrust) {
                reduction = highestThrustFound - maxAllowedThrust;
            }

            for (uint8_t k = 0; k < count; k++) {
                float thrustCappedUpper = uncapped[k] - reduction;
                motorvals[k] = capMinThrust(thrustCappedUpper);
            }
        }

        static uint16_t capMinThrust(float thrust) 
        {
            return thrust < 0 ? 0 : thrust;
        }

};
