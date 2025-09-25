#include <TinyPICO.h>

#include "task_helper.hpp"

class LedTask {

    public:

        void begin()
        {
            if (_task.didInit()){
                return;
            }

            _task.init(runLedTask, "led", this, 2);
        }

    private:

        static constexpr float HEARTBEAT_HZ = 1;
        static constexpr uint32_t PULSE_MSEC = 50;

        FreeRtosTask _task;

        TinyPICO tinypico = TinyPICO();

        static void runLedTask(void * obj)
        {
            ((LedTask *)obj)->run();
        }

        void run() 
        {
            TickType_t lastWakeTime = xTaskGetTickCount();

            while (true) {

                blink(lastWakeTime, HEARTBEAT_HZ);
            }
        }

        void blink(TickType_t & lastWakeTime, const float rate)
        {
            device_set(true);
            vTaskDelay(PULSE_MSEC);
            device_set(false);
            vTaskDelayUntil(&lastWakeTime, 1000/rate);
        }

        void device_init();

        void device_set(const bool on);
};
