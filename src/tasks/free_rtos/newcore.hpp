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

        void run(void)
        {
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
