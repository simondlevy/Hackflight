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
#include <console.h>
#include <datatypes.h>
#include <mixer.hpp>
#include <num.hpp>

#include <closedloops/altitude.hpp>
#include <closedloops/pitchroll_angle.hpp>
#include <closedloops/pitchroll_rate.hpp>
#include <closedloops/position.hpp>
#include <closedloops/yaw_angle.hpp>
#include <closedloops/yaw_rate.hpp>

class Hackflight {

    public:

        void init(
                const Clock::rate_t pidUpdateRate,
                const float thrustScale,
                const float thrustBase,
                const float thrustMin,
                const float thrustMax,
                const float pitchRollScale=1,
                const float yawScale=1)
        {
            _thrustScale = thrustScale;
            _thrustBase = thrustBase;
            _thrustMin = thrustMin;
            _thrustMax = thrustMax;
            _pitchRollScale = pitchRollScale;
            _yawScale = yawScale;

            initClosedLoopControllers(pidUpdateRate);

            _mixer.init();
        }

        void runClosedLoop(
                const bool inHoverMode,
                const vehicleState_t & vehicleState,
                demands_t & demands)
        {
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
            }

            _pitchRollAngleController.run(vehicleState, demands);

            _pitchRollRateController.run(vehicleState, demands);

            _yawAngleController.run(vehicleState, demands);

            _yawRateController.run(vehicleState, demands);
        }

        void runMixer(const demands_t demands, float motorvals[])
        {
            _mixer.run(demands, motorvals);
        }

    private:

        float _thrustScale;
        float _thrustBase;
        float _thrustMin;
        float _thrustMax;
        float _pitchRollScale;
        float _yawScale;

        Mixer _mixer;

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
};
