/**
 * Copyright (C) 2011-2018 Bitcraze AB, 2025 Simon D. Levy
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

#include <clock.hpp>
#include <__control__.hpp>
#include <kalman.hpp>
#include <datatypes.h>
#include <num.hpp>
#include <rateSupervisor.hpp>
#include <safety.hpp>
#include <task.hpp>
#include <tasks/debug.hpp>
#include <tasks/estimator.hpp>
#include <tasks/imu.hpp>
#include <tasks/setpoint.hpp>
#include <tasks/motors.hpp>
#include <vehicles/diyquad.hpp>

class CoreTask {

    public:

        void begin(
                ClosedLoopControl * closedLoopControl,
                Safety * safety,
                EstimatorTask * estimatorTask,
                ImuTask * imuTask,
                SetpointTask * setpointTask,
                MotorsTask * motorsTask,
                const uint8_t rotorCount,
                const mixFun_t mixFun,
				DebugTask * debugTask=nullptr)
        {
            _closedLoopControl = closedLoopControl;

            _safety = safety;

            _estimatorTask = estimatorTask;

            _imuTask = imuTask;

            _setpointTask = setpointTask;

            _motorsTask = motorsTask;

            _debugTask = debugTask;

            _mixFun = mixFun;

            _rotorCount = rotorCount;

            _task.init(runCoreTask, "core", this, 5);
        }

    private:

        static constexpr float LANDING_ALTITUDE_M = 0.03;

        static const auto PID_UPDATE_RATE = Clock::RATE_500_HZ;

        static const uint8_t TASK_ID_NBR = 3;

        static const uint32_t SETPOINT_TIMEOUT_TICKS = 1000;

        static const uint8_t MAX_MOTOR_COUNT = 20; // whatevs

        FreeRtosTask _task;

        uint8_t _rotorCount;

        void runMixer(const mixFun_t mixFun, const demands_t & demands,
                float motorvals[])
        {
            float uncapped[MAX_MOTOR_COUNT] = {};
            mixFun(demands, uncapped);

            float highestThrustFound = 0;
            for (uint8_t k=0; k<_rotorCount; k++) {

                const auto thrust = uncapped[k];

                if (thrust > highestThrustFound) {
                    highestThrustFound = thrust;
                }
            }

            float reduction = 0;
            if (highestThrustFound > THRUST_MAX) {
                reduction = highestThrustFound - THRUST_MAX;
            }

            for (uint8_t k = 0; k < _rotorCount; k++) {
                float thrustCappedUpper = uncapped[k] - reduction;
                motorvals[k] = capMinThrust(thrustCappedUpper);
            }
        }

        static uint16_t capMinThrust(float thrust) 
        {
            return thrust < 0 ? 0 : thrust;
        }

        vehicleState_t vehicleState;

        demands_t _demands;

        ClosedLoopControl * _closedLoopControl;

        SetpointTask * _setpointTask;

        MotorsTask * _motorsTask;

        EstimatorTask * _estimatorTask;

        ImuTask * _imuTask;

        DebugTask * _debugTask;

        Safety * _safety;

        mixFun_t _mixFun;

        static void runCoreTask(void* obj)
        {
            ((CoreTask *)obj)->run();
        }

        /* The core loop runs at 1kHz. It is the responsibility of the
         * different functions to run slower by skipping call (ie. returning
         * without modifying the output structure).
         */

        void run(void)
        {
            // Wait for sensors to be calibrated
            auto lastWakeTime = xTaskGetTickCount();
            while (!_imuTask->imuIsCalibrated()) {
                vTaskDelayUntil(&lastWakeTime, 1000/Clock::RATE_MAIN_LOOP);
            }

            static RateSupervisor rateSupervisor;
            rateSupervisor.init(xTaskGetTickCount(), 1000, 997, 1003, 1);

            uint32_t setpoint_timestamp = 0;
            bool lost_contact = false;

            for (uint32_t step=1; ; step++) {

                // The IMU should unlock at 1kHz
                _imuTask->waitDataReady();

                // Get state from estimator
                _estimatorTask->getVehicleState(&vehicleState);

                static float _motorvals[4];

                if (Clock::rateDoExecute(PID_UPDATE_RATE, step)) {

                    setpoint_t setpoint = {};

                    _setpointTask->getSetpoint(setpoint);

                    setpoint_timestamp = setpoint.timestamp;

                    if (setpoint.hovering) {

                        setpoint.demands.thrust = Num::rescale(
                                setpoint.demands.thrust, 0.2, 2.0, -1, +1);
                    }

                    // Use safety algorithm to modify demands based on sensor
                    // data and open-loop info
                    _safety->update(step, setpoint.timestamp, vehicleState);

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
                            vehicleState,
                            setpoint.demands,
                            LANDING_ALTITUDE_M,
                            closedLoopDemands);

                    runMixer(_mixFun, closedLoopDemands, _motorvals);
                }

                const auto timestamp = xTaskGetTickCount();

                // Disarm on lost contact
                if (setpoint_timestamp > 0 &&
                        timestamp - setpoint_timestamp >
                        SETPOINT_TIMEOUT_TICKS) {
                    lost_contact = true;
                    _motorsTask->stop();
                    _safety->requestArming(false);
                }

                else if (!lost_contact && _safety->isArmed()) {
                
                    const uint16_t motorsPwm[4]  = {
                        (uint16_t)_motorvals[0],
                        (uint16_t)_motorvals[1],
                        (uint16_t)_motorvals[2],
                        (uint16_t)_motorvals[3]
                    };

                    _motorsTask->setRatios(motorsPwm);
                } 

                else {
                    _motorsTask->stop();
                }

                if (!rateSupervisor.validate(timestamp)) {
                    static bool rateWarningDisplayed;
                    if (!rateWarningDisplayed) {
                        rateWarningDisplayed = true;
                    }
                }
            }
        }
};
