/**
 * Copyright (C) 2025 Simon D. Levy
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

#include <__control__.hpp>
#include <safety.hpp>
#include <task.hpp>
#include <tasks/debug.hpp>
#include <tasks/estimator.hpp>
#include <tasks/imu.hpp>
#include <tasks/setpoint.hpp>

class CoreTask {

    public:

        void begin(
                ClosedLoopControl * closedLoopControl,
                Safety * safety,
                EstimatorTask * estimatorTask,
                ImuTask * imuTask,
                SetpointTask * setpointTask,
                const uint8_t motorCount,
                const mixFun_t mixFun,
				DebugTask * debugTask=nullptr)
        {
            _closedLoopControl = closedLoopControl;

            _safety = safety;

            _estimatorTask = estimatorTask;

            _imuTask = imuTask;

            _setpointTask = setpointTask;

            _debugTask = debugTask;

            _mixFun = mixFun;

            _motorCount = motorCount;

            _task.init(runCoreTask, "core", this, 5);
        }

    private:

        static const auto PID_UPDATE_RATE = Clock::RATE_500_HZ;
        static constexpr float LANDING_ALTITUDE_M = 0.03;
        static const uint8_t MAX_MOTOR_COUNT = 20; // whatevs
        static const uint32_t SETPOINT_TIMEOUT_TICKS = 1000;

        ClosedLoopControl * _closedLoopControl;
        SetpointTask * _setpointTask;
        EstimatorTask * _estimatorTask;
        ImuTask * _imuTask;
        DebugTask * _debugTask;
        Safety * _safety;
        mixFun_t _mixFun;
        uint8_t _motorCount;
        FreeRtosTask _task;
        uint16_t _motorRatios[4];
        vehicleState_t _vehicleState;

        static void runCoreTask(void *arg)
        {
            ((CoreTask *)arg)->run();
        }

        void setMotorRatiosAndRun(const uint16_t ratios[])
        {
            setMotorRatio(0, ratios[0]);
            setMotorRatio(1, ratios[1]);
            setMotorRatio(2, ratios[2]);
            setMotorRatio(3, ratios[3]);

            // Device-dependent
            motors_run();
        }

        void run()
        {
            // Device-dependent
            motors_init();

            idleMotors();

            static RateSupervisor rateSupervisor;
            rateSupervisor.init(xTaskGetTickCount(), 1000, 997, 1003, 1);

            uint32_t setpoint_timestamp = 0;
            bool lost_contact = false;

            for (uint32_t step=1; ; step++) {

                setMotorRatiosAndRun(_motorRatios);

                _imuTask->waitDataReady();

                _estimatorTask->getVehicleState(&_vehicleState);

                static float _motorvals[MAX_MOTOR_COUNT];

                if (Clock::rateDoExecute(PID_UPDATE_RATE, step)) {
                    runPidControlAndMixer(
                            step, setpoint_timestamp, _motorvals);
                }

                const auto timestamp = xTaskGetTickCount();

                // Disarm on lost contact
                if (setpoint_timestamp > 0 &&
                        timestamp - setpoint_timestamp >
                        SETPOINT_TIMEOUT_TICKS) {
                    lost_contact = true;
                    idleMotors();
                    _safety->requestArming(false);
                }

                // Run in flying mode
                else if (!lost_contact && _safety->isArmed()) {
                    runMotors(_motorvals);
                } 

                // Otherwise, maintain motors at stopped (idle) values
                else {
                    idleMotors();
                }

                if (!rateSupervisor.validate(timestamp)) {
                    static bool rateWarningDisplayed;
                    if (!rateWarningDisplayed) {
                        rateWarningDisplayed = true;
                    }
                }
            }
        }

        void runMixer(const mixFun_t mixFun, const demands_t & demands,
                float motorvals[])
        {
            float uncapped[MAX_MOTOR_COUNT] = {};
            mixFun(demands, uncapped);

            float highestThrustFound = 0;
            for (uint8_t k=0; k<_motorCount; k++) {

                const auto thrust = uncapped[k];

                if (thrust > highestThrustFound) {
                    highestThrustFound = thrust;
                }
            }

            float reduction = 0;
            if (highestThrustFound > THRUST_MAX) {
                reduction = highestThrustFound - THRUST_MAX;
            }

            for (uint8_t k = 0; k < _motorCount; k++) {
                float thrustCappedUpper = uncapped[k] - reduction;
                motorvals[k] = capMinThrust(thrustCappedUpper);
            }
        }

        static uint16_t capMinThrust(float thrust) 
        {
            return thrust < 0 ? 0 : thrust;
        }

        void setMotorRatio(uint32_t id, uint16_t ratio)
        {
            // Device-dependent
            motors_setSpeed(id, ratio/65536.f);

            static uint16_t m1, m2, m3;

            if (id == 0) m1 = ratio;
            if (id == 1) m2 = ratio;
            if (id == 2) m3 = ratio;
        }

        void runMotors(const float motorvals[4]) 
        {
            const uint16_t motorsPwm[4]  = {
                (uint16_t)motorvals[0],
                (uint16_t)motorvals[1],
                (uint16_t)motorvals[2],
                (uint16_t)motorvals[3]
            };

            setMotorRatios(motorsPwm);
        }

        void setMotorRatios(const uint16_t ratios[4])
        {
            _motorRatios[0] = ratios[0];
            _motorRatios[1] = ratios[1];
            _motorRatios[2] = ratios[2];
            _motorRatios[3] = ratios[3];
        }

        void idleMotors()
        {
            const uint16_t ratios[4] = {0, 0, 0, 0};
            setMotorRatiosAndRun(ratios);
        }

        void runPidControlAndMixer(
                const uint32_t step,
                uint32_t & setpoint_timestamp,
                float * motorvals)
        {
            setpoint_t setpoint = {};

            _setpointTask->getSetpoint(setpoint);

            setpoint_timestamp = setpoint.timestamp;

            if (setpoint.hovering) {

                setpoint.demands.thrust = Num::rescale(
                        setpoint.demands.thrust, 0.2, 2.0, -1, +1);
            }

            // Use safety algorithm to modify demands based on current
            // vehicle state, open-loop demads, and motor values
            _safety->update(step, setpoint.timestamp, _vehicleState,
                    _motorRatios, _motorCount);

            if (setpoint.hovering) {

                // In hover mode, thrust demand comes in as [-1,+1], so
                // we convert it to a target altitude in meters
                setpoint.demands.thrust = Num::rescale(
                        setpoint.demands.thrust, -1, +1, 0.2, 2.0);
            }

            demands_t closedLoopDemands = {};

            _closedLoopControl->run(
                    1.f / PID_UPDATE_RATE,
                    setpoint.hovering,
                    _vehicleState,
                    setpoint.demands,
                    LANDING_ALTITUDE_M,
                    closedLoopDemands);

            runMixer(_mixFun, closedLoopDemands, motorvals);
        }

        // Device-dependent ---------------------------

        void motors_init();

        void motors_setSpeed(uint32_t id, float speed);

        void motors_run();
};
