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
#include <rateSupervisor.hpp>
#include <safety.hpp>

class CoreTask : public FreeRTOSTask {

    public:

        // Shared with logger or params
        vehicleState_t vehicleState;
        Safety safety;
        EstimatorTask estimatorTask;
        FlowDeckTask flowDeckTask;
        ZRangerTask zrangerTask;

        void begin(
                const float rollCalibration,
                const float pitchCalibration,
                const uint8_t flowDeckCsPin,
                VL53L1 * vl53l1,
                const openLoopFun_t openLoopFun,
                const mixFun_t mixFun,
                const bool isTeensy=false)
        {
            FreeRTOSTask::begin(runCoreTask, "CORE", this, 5);
        }

    private:

        static void runCoreTask(void* obj)
        {
            ((CoreTask *)obj)->run();
        }

        // Approximate thrust needed when in perfect hover. More weight/older
        // battery can use a higher value
        static constexpr float THRUST_BASE  = 36000;
        static constexpr float THRUST_MIN   = 20000;
        static constexpr float THRUST_SCALE = 1000;

        static constexpr float THRUST_MAX = UINT16_MAX;

        static const auto PID_UPDATE_RATE = Clock::RATE_500_HZ;

        Hackflight _hackflight;

        demands_t _demands;

        openLoopFun_t _openLoopFun;

        ImuTask _imuTask;


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
                break;
            }
            
            consolePrintf("CORE: Starting loop\n");
            rateSupervisor.init(xTaskGetTickCount(), M2T(1000), 997, 1003, 1);

            while (true) {

                static uint32_t _prev;
                static uint8_t _on;

                auto msec = millis();

                if (msec - _prev > 500) {

                    digitalWriteFast(LED_BUILTIN, _on);
                    _on = !_on;
                    _prev = msec;
                }

                delay(1);
            }        
        }

};
