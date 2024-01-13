#pragma once

#include <console.h>
#include <task.hpp>

class ReceiverTask : public FreeRTOSTask {

    public:


        void begin(void)
        {
            pinMode(LED_BUILTIN, OUTPUT);

            FreeRTOSTask::begin(run, "receiever", this, 2);
        }

        void getRawChannelValues(int16_t chanvals[6])
        {
            chanvals[0] = 1000;
            chanvals[1] = 1200;
            chanvals[2] = 1400;
            chanvals[3] = 1600;
            chanvals[4] = 1800;
            chanvals[5] = 2000;
        }

    private:

        static void run(void * obj) 
        {
            ((ReceiverTask *)obj)->run();
        }

        void run(void)
        {
            consolePrintf("RECEIVER: running\n");

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
