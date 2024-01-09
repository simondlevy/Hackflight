#pragma once

#include <math.h>

#include <free_rtos.h>
#include <task.h>

#include <tasks/estimator.hpp>
#include <tasks/imu.hpp>

#include <crossplatform.h>
#include <hackflight.hpp>
#include <kalman.hpp>
#include <motors.h>
#include <openloop.hpp>
#include <rateSupervisor.hpp>
#include <safety.hpp>
#include <system.h>

class CoreTask {

    public:

        // Shared with logger or params
        vehicleState_t state;
        Safety safety;

        void init(
                OpenLoop * openLoop,
                ImuTask * imuTask,
                EstimatorTask * estimatorTask,
                const mixfun_t mixfun)
        {
            if (_didInit) {
                return;
            }

            safety.init();

            _openLoop = openLoop;
            _imuTask = imuTask;
            _estimatorTask = estimatorTask;

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
            _hackflight.resetControllers();
        }

        void handleImuDataAvailable(void)
        {
            _imuTask->dataAvailableCallback();
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

        Hackflight _hackflight;

        demands_t _demands;

        OpenLoop * _openLoop;
        EstimatorTask * _estimatorTask;
        ImuTask * _imuTask;

        bool _didInit = false;

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
            while(!_imuTask->areCalibrated()) {
                vTaskDelayUntil(&lastWakeTime, F2T(Clock::RATE_MAIN_LOOP));
            }
            // Initialize step to something else than 0
            uint32_t step = 1;

            systemWaitStart();
            consolePrintf("CORE: Starting loop\n");
            rateSupervisor.init(xTaskGetTickCount(), M2T(1000), 997, 1003, 1);

            while (true) {

                // The IMU should unlock at 1kHz
                _imuTask->waitDataReady();
                sensorData_t sensorData = {};
                _imuTask->acquire(&sensorData);

                // Get state vector linear positions and velocities and
                // angles from estimator
                _estimatorTask->getVehicleState(&state);

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
