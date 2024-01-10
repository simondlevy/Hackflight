/**
 * Copyright (C) 2011-2018 Bitcraze AB, 2024 Simon D. Levy
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

#include <free_rtos.h>
#include <task.h>

#include <vl53l1.hpp>

#include <tasks/free_rtos.hpp>
#include <tasks/free_rtos/estimator.hpp>
#include <tasks/free_rtos/flowdeck.hpp>
#include <tasks/free_rtos/imu.hpp>
#include <tasks/free_rtos/zranger.hpp>

#include <crossplatform.h>
#include <hackflight.hpp>
#include <kalman.hpp>
#include <motors.h>
#include <openloop.hpp>
#include <rateSupervisor.hpp>
#include <safety.hpp>

class CoreTask : public FreeRTOSTask {

    public:

        // Shared with logger or params
        vehicleState_t state;
        Safety safety;
        EstimatorTask estimatorTask;
        FlowDeckTask flowDeckTask;
        ZRangerTask zrangerTask;

        void init(
                const float rollCalibration,
                const float pitchCalibration,
                const uint8_t flowDeckCsPin,
                VL53L1 * vl53l1,
                OpenLoop * openLoop,
                const mixfun_t mixfun)
        {
            if (didInit) {
                return;
            }

            estimatorTask.init(&safety);

            flowDeckTask.init(flowDeckCsPin, &estimatorTask);

            zrangerTask.init(vl53l1, &estimatorTask);

            safety.init();

            _imuTask.init(&estimatorTask, rollCalibration, pitchCalibration);

            _openLoop = openLoop;

            _hackflight.init(
                    mixfun,
                    PID_UPDATE_RATE,
                    THRUST_SCALE,
                    THRUST_BASE,
                    THRUST_MIN,
                    THRUST_MAX
                    );

            motorsInit();

            xTaskCreateStatic(
                    runTask, 
                    "CORE", 
                    STACKSIZE,
                    this, 
                    5, 
                    taskStackBuffer,
                    &taskTaskBuffer);
        }


        bool test(void)
        {
            auto pass = true;

            pass &= _imuTask.test();
            pass &= estimatorTask.didInit();
            pass &= motorsTest();

            return pass;
        }

        void resetControllers(void)
        {
            _hackflight.resetControllers();
        }

        void handleImuDataAvailable(void)
        {
            _imuTask.dataAvailableCallback();
        }

    private:

        // approximate thrust needed when in perfect hover. More weight/older
        // battery can use a higher value
        static constexpr float THRUST_BASE  = 36000;
        static constexpr float THRUST_MIN   = 20000;
        static constexpr float THRUST_SCALE = 1000;

        static constexpr float THRUST_MAX = UINT16_MAX;

        static const auto PID_UPDATE_RATE = Clock::RATE_500_HZ;

        static const auto STACKSIZE = 3* configMINIMAL_STACK_SIZE;
        StackType_t  taskStackBuffer[STACKSIZE]; 
        StaticTask_t taskTaskBuffer;

        Hackflight _hackflight;

        demands_t _demands;

        OpenLoop * _openLoop;

        ImuTask _imuTask;

        void runMotors(const float motorvals[4]) 
        {
            const uint16_t motorsPwm[4]  = {
                (uint16_t)motorvals[0],
                (uint16_t)motorvals[1],
                (uint16_t)motorvals[2],
                (uint16_t)motorvals[3]
            };

            motorsSetRatios(motorsPwm);
        }

        static void runTask(void* param)
        {
            ((CoreTask *)param)->run();
        }

        /* The core loop runs at 1kHz. It is the
         * responsibility of the different functions to run slower by skipping call
         * (ie. returning without modifying the output structure).
         */
        void run(void)
        {
            static RateSupervisor rateSupervisor;

            vTaskSetApplicationTaskTag(0, (TaskHookFunction_t)TASK_CORE_ID_NBR);

            //Wait for the system to be fully started to start core loop
            systemWaitStart();

            consolePrintf("CORE: Wait for sensor calibration...\n");

            // Wait for sensors to be calibrated
            auto lastWakeTime = xTaskGetTickCount();
            while(!_imuTask.areCalibrated()) {
                vTaskDelayUntil(&lastWakeTime, F2T(Clock::RATE_MAIN_LOOP));
            }
            consolePrintf("CORE: Starting loop\n");
            rateSupervisor.init(xTaskGetTickCount(), M2T(1000), 997, 1003, 1);

            for (uint32_t step=1; ; step++) {

                // The IMU should unlock at 1kHz
                _imuTask.waitDataReady();
                sensorData_t sensorData = {};
                _imuTask.acquire(&sensorData);

                // Get state vector linear positions and velocities and
                // angles from estimator
                estimatorTask.getVehicleState(&state);

                // Get state vector angular velocities directly from gyro
                state.dphi =    sensorData.gyro.x;     
                state.dtheta = -sensorData.gyro.y; // (negate for ENU)
                state.dpsi =    sensorData.gyro.z; 

                const auto areMotorsAllowedToRun = safety.areMotorsAllowedToRun();

                static float _motorvals[4];

                if (Clock::rateDoExecute(PID_UPDATE_RATE, step)) {

                    uint32_t timestamp = 0;
                    auto inHoverMode = false;

                    // Get open-loop demands in [-1,+1], as well as timestamp
                    // when they received, and whether hover mode is indicated
                    _openLoop->getDemands(_demands, timestamp, inHoverMode);

                    // Use safety algorithm to modify demands based on sensor data
                    // and open-loop info
                    safety.update(sensorData, step, timestamp, _demands);

                    // Run hackflight core algorithm to get motor spins from open
                    // loop demands via closed-loop control and mixer
                    _hackflight.step(inHoverMode, state, _demands, _motorvals);
                }

                if (areMotorsAllowedToRun) {
                    runMotors(_motorvals);
                } else {
                    motorsStop();
                }

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
};
