/**
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

        // Shared with logger
        vehicleState_t vehicleState;

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

            setStatus(STATUS_DISARMED);

            initClosedLoopControllers(pidUpdateRate);

            _mixer.init();
        }

        void runClosedLoop(
                const demands_t & openLoopDemands,
                const vehicleState_t & vehicleState)
        {
            // Start with open-loop demands
            memcpy(&_demands, &openLoopDemands, sizeof(demands_t));

            // Run throttle and yaw demandthrough deadband
            _demands.thrust = deadband(_demands.thrust);
            _demands.yaw = deadband(_demands.yaw);

            // If armed, transition to takeoff when throttle stick moves
            // into deadband
            if (_status == STATUS_ARMED && inband(_demands.thrust)) {
                setStatus(STATUS_TAKEOFF);
            }

            // If in takeoff and initial target altitude reached, switch status
            // to flying
            if (_status == STATUS_TAKEOFF && 
                    vehicleState.z > INITIAL_ALTITUDE_TARGET) {
                setStatus(STATUS_FLYING);
            }

            // If we're flying and altitude drops below a minimum, set thrust to 
            // minimum and status to landed
            if (_status == STATUS_FLYING) {

                if (vehicleState.z < LANDING_ALTITUDE) {
                    _demands.thrust = -1;
                    setStatus(STATUS_ARMED);
                }
            }

            // If we're flying or taking off, run the closed-loop controllers
            // to get the final demands
            if (_status == STATUS_FLYING || _status == STATUS_TAKEOFF) {

                // Run closed-loop controllers to update demands
                runClosedLoopControllers(vehicleState);

                // Scale demands for motors
                scaleDemands();
            }
        }

        void runMixer(float motorvals[])
        {
            _mixer.run(_demands, motorvals);
        }

        //////////////////////////////////////////////////////////////////////

        bool isDisarmed(void)
        {
            return _status == STATUS_DISARMED;
        }

        void setStatus(const flightStatus_e status)
        {
            _status = status;

            static const char * labels[4] = {
                "disarmed", "armed", "takeoff", "flying"
            };

            consolePrintf("STATUS: %s\n", labels[status]);
        }

        void resetDemands(void)
        {
            _demands.roll = 0;
            _demands.pitch = 0;
            _demands.yaw = 0;
        }

        void capMotors(float motorsUncapped[], uint16_t motorsPwm[])
        {
            _mixer.capMotors(motorsUncapped, motorsPwm);
        }


        // Called by estimator task
        bool isFlying(void)
        {
            return _status == STATUS_FLYING || _status == STATUS_TAKEOFF;
        }

        void resetClosedLoopControllers(void)
        {
            _pitchRollAngleController.resetPids();
            _pitchRollRateController.resetPids();
            _positionController.resetPids();

            _altitudeController.resetFilters();
            _positionController.resetFilters();
        }

    private:

        static constexpr float STICK_DEADBAND = 0.25;

        static constexpr float INITIAL_ALTITUDE_TARGET = 0.4;

        static constexpr float LANDING_ALTITUDE = 0.04;

        //////////////////////////////////////////////////////////////////////

        static float deadband(const float stickValue)
        {
            return inband(stickValue) ? 0 : stickValue;
        }

        static float inband(const float stickValue)
        {
            return fabs(stickValue) < STICK_DEADBAND;
        }

        //////////////////////////////////////////////////////////////////////

        float _thrustScale;
        float _thrustBase;
        float _thrustMin;
        float _thrustMax;
        float _pitchRollScale;
        float _yawScale;

        flightStatus_e _status;

        demands_t _demands;

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
            _positionController.init(pidUpdateRate, INITIAL_ALTITUDE_TARGET);
            _altitudeController.init(pidUpdateRate, INITIAL_ALTITUDE_TARGET);
        }

        void runClosedLoopControllers(const vehicleState_t vehicleState)
        {
            // Altitude controller converts throttle demand into unscaled
            // motors spin
            _altitudeController.run(vehicleState, _demands);

            // Position controller converts meters per second to
            // angles in degrees
            _positionController.run(vehicleState, _demands); 

            // Pitch/roll angle controller converts angles in degrees
            // into angular rates in degrees per second
            _pitchRollAngleController.run(vehicleState, _demands);

            // Pitch/roll rate controller converts angular rates in
            // degrees per second into unscaled motor spin
            _pitchRollRateController.run(vehicleState, _demands);

            // Yaw angle controller converts [-1,+1] stick demand into angular
            // rate in degrees per second
            _yawAngleController.run(vehicleState, _demands);

            // Yaw rate controller converts angular rate into 
            // unscaled motor spins
            _yawRateController.run(vehicleState, _demands);
        }

        void scaleDemands(void)
        {
            _demands.thrust = Num::fconstrain(
                    _demands.thrust * _thrustScale + _thrustBase,
                    _thrustMin, _thrustMax);

            _demands.roll *= _pitchRollScale;

            _demands.pitch *= _pitchRollScale;

            _demands.yaw *= _yawScale;
        }
};
