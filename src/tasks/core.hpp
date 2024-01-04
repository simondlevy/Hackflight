/**
 * Copyright (C) 2011-2022 Bitcraze AB, 2024 Simon D. Levy
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

#include <free_rtos.h>
#include <task.h>

// Arduino class
#include <vl53l1.hpp>

#include <hackflight.hpp>

#include <tasks/estimator.hpp>
#include <tasks/flowdeck.hpp>
#include <tasks/imu.hpp>
#include <tasks/rx.hpp>
#include <tasks/zranger.hpp>

#include <crossplatform.h>
#include <motors.h>
#include <rateSupervisor.hpp>
#include <system.h>

class CoreTask {

    public:

        // Shared with logger
        vehicleState_t vehicleState;

        void init(
                const float rollCalibration,
                const float pitchCalibration,
                VL53L1 * vl53l1)
        {
            if (_didInit) {
                return;
            }

            _hackflight.init(
                    PID_UPDATE_RATE,
                    THRUST_SCALE,
                    THRUST_BASE,
                    THRUST_MIN,
                    THRUST_MAX);

            // Prevents arming until aux switch is off
            _wasAuxSet = true;

            _flowDeckTask.init(&_estimatorTask);

            _imuTask.init(&_estimatorTask, rollCalibration, pitchCalibration);

            _estimatorTask.init();

            _zrangerTask.init(vl53l1, &_estimatorTask);

            _rxTask.init();

            motorsInit();

            xTaskCreateStatic(
                    runTask, 
                    "CORE", 
                    TASK_STACK_DEPTH,
                    this, 
                    5, 
                    taskStackBuffer,
                    &taskTaskBuffer);

            _didInit = true;
        }

        bool test(void)
        {
            auto pass = true;

            pass &= _imuTask.test();
            pass &= _estimatorTask.didInit();
            pass &= motorsTest();

            return pass;
        }

        // Called by estimator task
        bool isFlying(void)
        {
            return _hackflight.isFlying();
        }

        void resetControllers(void)
        {
            _hackflight.resetClosedLoopControllers();
        }

    private:

        static constexpr float THROTTLE_DOWN_TOLERANCE = 0.01;

        static constexpr float MAX_ARMING_ANGLE_DEGREES = 25;

        // approximate thrust needed when in perfect hover. More weight/older
        // battery can use a higher value
        static constexpr float THRUST_BASE  = 36000;
        static constexpr float THRUST_MIN   = 20000;
        static constexpr float THRUST_SCALE = 1000;
        static constexpr float THRUST_MAX   = UINT16_MAX;

        static const uint8_t MAX_MOTOR_COUNT = 20;

        static const auto PID_UPDATE_RATE = Clock::RATE_500_HZ;

        Hackflight _hackflight;

        static const auto TASK_STACK_DEPTH = 3* configMINIMAL_STACK_SIZE;
        StackType_t  taskStackBuffer[TASK_STACK_DEPTH]; 
        StaticTask_t taskTaskBuffer;

        EstimatorTask _estimatorTask;
        ZRangerTask _zrangerTask;
        FlowDeckTask _flowDeckTask;
        RxTask _rxTask;
        ImuTask _imuTask;

        bool _didInit;

        bool _wasAuxSet;

        static void runTask(void* param)
        {
            ((CoreTask *)param)->run();
        }

        static bool isShallowAngle(const float degrees)
        {
            return fabs(degrees) < MAX_ARMING_ANGLE_DEGREES;
        }

        /* The core loop runs at 1kHz. It is the
         * responsibility of the different functions to run slower by skipping call
         * (ie. returning without modifying the output structure).
         */
        void run(void)
        {
            static RateSupervisor rateSupervisor;

            vTaskSetApplicationTaskTag(0, (TaskHookFunction_t)TASK_CORE_ID_NBR);

            // Wait for the system to be fully started to start core loop
            systemWaitStart();

            consolePrintf("CORE: Wait for sensor calibration...\n");

            // Wait for sensors to be calibrated
            auto lastWakeTime = xTaskGetTickCount();
            while(!_imuTask.areCalibrated()) {
                vTaskDelayUntil(&lastWakeTime, F2T(Clock::RATE_MAIN_LOOP));
            }
            systemWaitStart();
            consolePrintf("CORE: Starting loop\n");
            rateSupervisor.init(xTaskGetTickCount(), M2T(1000), 997, 1003, 1);

            for (uint32_t step=1; ; step++) {

                // Reset everything on throttle stick down
                if (isThrottleDown()) {
                    _hackflight.resetDemands();
                    _hackflight.resetClosedLoopControllers();
                }

                // The sensor should unlock at 1kHz
                _imuTask.waitDataReady();

                // Get state vector linear positions and velocities and
                // angles from estimator
                _estimatorTask.getVehicleState(&vehicleState);

                // Run closed-loop controllers periodically
                if (Clock::rateDoExecute(PID_UPDATE_RATE, step)) {

                    // Use safety algorithm to enable/disable arming demands
                    // based on vehicle state and RX activity
                    updateSafety();

                    _hackflight.runClosedLoop(_rxTask.demands, vehicleState);
                }

                // Start with motors zeroed-out for safety
                uint16_t motorsPwm[MAX_MOTOR_COUNT] = {};

                // Run mixer and motors if safe
                if (_hackflight.isFlying()) {

                    float motorsUncapped[MAX_MOTOR_COUNT] = {};

                    _hackflight.runMixer(motorsUncapped);

                    _hackflight.capMotors(motorsUncapped, motorsPwm);
                } 

                motorsSetRatios(motorsPwm);                

                if (!rateSupervisor.validate(xTaskGetTickCount())) {
                    static bool rateWarningDisplayed;
                    if (!rateWarningDisplayed) {
                        consolePrintf("CORE: WARNING: loop rate is off (%lu)\n", 
                                rateSupervisor.getLatestCount());
                        rateWarningDisplayed = true;
                    }
                }

                // motorsCheckDshot();
            }
        }

        bool isThrottleDown(void)
        {
            return _rxTask.demands.thrust < -(1 - THROTTLE_DOWN_TOLERANCE);
        }

        void updateSafety(void)
        { 
            const auto isAuxSet = _rxTask.isAuxSet;

            // If aux switch is off, disarm
            if (!isAuxSet) {

                if (!_hackflight.isDisarmed()) {
                    _hackflight.setStatus(STATUS_DISARMED);
                }
            }

            // If aux switch turns on and throttle is down and pitch/roll angles are
            // shallow, we can arm
            else if (
                    !_wasAuxSet && 
                    isThrottleDown() &&
                    isShallowAngle(vehicleState.phi) &&
                    isShallowAngle(vehicleState.theta)
                    ) {

                _hackflight.setStatus(STATUS_ARMED);
            }

            // Track previous aux switch state to detect transition
            _wasAuxSet = isAuxSet;
        }
};
