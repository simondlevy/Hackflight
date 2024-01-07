#pragma once

#include <math.h>

#include <free_rtos.h>
#include <task.h>

#include <closedloops/altitude.hpp>
#include <closedloops/pitchroll_angle.hpp>
#include <closedloops/pitchroll_rate.hpp>
#include <closedloops/position.hpp>
#include <closedloops/yaw_angle.hpp>
#include <closedloops/yaw_rate.hpp>

#include <tasks/estimator.hpp>
#include <tasks/imu.hpp>

#include <crossplatform.h>
#include <kalman.hpp>
#include <mixer.hpp>
#include <motors.h>
#include <openloop.hpp>
#include <rateSupervisor.hpp>
#include <safety.hpp>
#include <system.h>

class CoreTask {

    public:

        // Shared with logger
        demands_t demands;
        vehicleState_t state;

        void init(
                OpenLoop * openLoop,
                ImuTask * imuTask,
                EstimatorTask * estimatorTask,
                Safety * safety)
        {
            if (_didInit) {
                return;
            }

            _openLoop = openLoop;
            _imuTask = imuTask;
            _estimatorTask = estimatorTask;
            _safety = safety;

            _mixer.init();

            _pitchRollAngleController.init(PID_UPDATE_RATE);
            _pitchRollRateController.init(PID_UPDATE_RATE);
            _yawAngleController.init(PID_UPDATE_RATE);
            _yawRateController.init(PID_UPDATE_RATE);
            _positionController.init(PID_UPDATE_RATE);
            _altitudeController.init(PID_UPDATE_RATE);

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

            pass &= _imuTask->test();
            pass &= _estimatorTask->didInit();
            pass &= motorsTest();

            return pass;
        }

        void resetControllers(void)
        {
            _pitchRollAngleController.resetPids();
            _pitchRollRateController.resetPids();
            _positionController.resetPids();

            _altitudeController.resetFilters();
            _positionController.resetFilters();
        }

    private:

        // approximate thrust needed when in perfect hover. More weight/older
        // battery can use a higher value
        static constexpr float THRUST_BASE  = 36000;
        static constexpr float THRUST_MIN   = 20000;
        static constexpr float THRUST_SCALE = 1000;

        static constexpr float THRUST_MAX = UINT16_MAX;

        static const auto PID_UPDATE_RATE = Clock::RATE_500_HZ;

        static const auto TASK_STACK_DEPTH = 3* configMINIMAL_STACK_SIZE;
        StackType_t  taskStackBuffer[TASK_STACK_DEPTH]; 
        StaticTask_t taskTaskBuffer;

        PitchRollAngleController _pitchRollAngleController;
        PitchRollRateController _pitchRollRateController;
        PositionController _positionController;
        AltitudeController _altitudeController;
        YawAngleController _yawAngleController;
        YawRateController _yawRateController;

        Mixer _mixer;

        OpenLoop * _openLoop;
        EstimatorTask * _estimatorTask;
        ImuTask * _imuTask;
        Safety * _safety;

        bool _didInit = false;

        void runMotors(void) 
        {
            float motorvals[4] = {};

            _mixer.run(demands, motorvals);

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
            while(!_imuTask->areCalibrated()) {
                vTaskDelayUntil(&lastWakeTime, F2T(Clock::RATE_MAIN_LOOP));
            }
            // Initialize step to something else than 0
            uint32_t step = 1;

            systemWaitStart();
            consolePrintf("CORE: Starting loop\n");
            rateSupervisor.init(xTaskGetTickCount(), M2T(1000), 997, 1003, 1);

            while (true) {

                // The sensor should unlock at 1kHz
                _imuTask->waitDataReady();
                sensorData_t sensorData = {};
                _imuTask->acquire(&sensorData);

                // Get state vector linear positions and velocities and
                // angles from estimator
                _estimatorTask->getState(&state);

                // Get state vector angular velocities directly from gyro
                state.dphi =    sensorData.gyro.x;     
                state.dtheta = -sensorData.gyro.y; // (negate for ENU)
                state.dpsi =    sensorData.gyro.z; 

                const auto areMotorsAllowedToRun = _safety->areMotorsAllowedToRun();

                // Run closed-loop controllers periodically
                if (Clock::rateDoExecute(PID_UPDATE_RATE, step)) {

                    uint32_t timestamp = 0;
                    auto inHoverMode = false;

                    // Get open-loop demands in [-1,+1], as well as timestamp
                    // when they received, and whether hover mode is indicated
                    _openLoop->getDemands(demands, timestamp, inHoverMode);

                    // Use safety algorithm to modify demands based on sensor data
                    // and open-loop info
                    _safety->update(sensorData, step, timestamp, demands);

                    if (inHoverMode) {

                        // In hover mode, thrust demand comes in as [-1,+1], so
                        // we convert it to a target altitude in meters
                        demands.thrust = Num::rescale(
                                demands.thrust, -1, +1, 0.2, 2.0);

                        // Position controller converts meters per second to
                        // degrees
                        _positionController.run(state, demands); 

                        _altitudeController.run(state, demands); 

                        // Scale up thrust demand for motors
                        demands.thrust = Num::fconstrain(
                                demands.thrust * THRUST_SCALE + THRUST_BASE,
                                THRUST_MIN, THRUST_MAX);
                    }

                    else {

                        // In non-hover mode, thrust demand comes in as [0,1],
                        // so we convert it to [0, 2^16] for motors
                        demands.thrust *= UINT16_MAX;

                        // In non-hover mode, pitch/roll demands come in as
                        // [-1,+1], which we convert to degrees for input to
                        // pitch/roll controller
                        demands.roll *= 30;
                        demands.pitch *= 30;
                    }

                    _pitchRollAngleController.run(state, demands);

                    _pitchRollRateController.run(state, demands);

                    _yawAngleController.run(state, demands);

                    _yawRateController.run(state, demands);
                }

                // Reset closed-loop controllers on zero thrust
                if (demands.thrust == 0) {

                    demands.roll = 0;
                    demands.pitch = 0;
                    demands.yaw = 0;

                    resetControllers();
                }

                // Critical for safety, be careful if you modify this code!
                // The safety will already set thrust to 0 if needed, but to be
                // extra sure prevent motors from running.
                if (areMotorsAllowedToRun) {
                    runMotors();
                } else {
                    motorsStop();
                }

                step++;

                if (!rateSupervisor.validate(xTaskGetTickCount())) {
                    static bool rateWarningDisplayed;
                    if (!rateWarningDisplayed) {
                        consolePrintf("CORE: WARNING: loop rate is off (%lu)\n", 
                                rateSupervisor.getLatestCount());
                        rateWarningDisplayed = true;
                    }
                }

                motorsCheckDshot();
            }
        }
};
