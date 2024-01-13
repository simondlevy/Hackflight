#pragma once

#include <console.h>
#include <tasks/free_rtos.hpp>

class BlinkyTask : public FreeRTOSTask {

    public:


        void begin(void)
        {
            pinMode(LED_BUILTIN, OUTPUT);

            FreeRTOSTask::begin(runBlinkyTask, "BlinkyTask", this, 2);
        }

    private:

        static void runBlinkyTask(void * obj) 
        {
            ((BlinkyTask *)obj)->run();
        }

        void run(void)
        {
            consolePrintf("BlinkyTask: running\n");

            while (true) {

                static uint32_t prev;

                auto msec = millis();

                if (msec - prev > 500) {

                    static bool toggle;

                    digitalWrite(LED_BUILTIN, toggle);

                    toggle = !toggle;

                    prev = msec;
                }

                vTaskDelay(1);

            }
        }
};
