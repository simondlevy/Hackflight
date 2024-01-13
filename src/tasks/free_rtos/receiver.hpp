#pragma once

#include <console.h>
#include <tasks/free_rtos.hpp>

class ReceiverTask : public FreeRTOSTask {

    public:


        void begin(void)
        {
            pinMode(LED_BUILTIN, OUTPUT);

            FreeRTOSTask::begin(run, "ReceiverTask", this, 2);
        }

    private:

        static void run(void * obj) 
        {
            ((ReceiverTask *)obj)->run();
        }

        void run(void)
        {
            consolePrintf("ReceiverTask: running\n");

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
