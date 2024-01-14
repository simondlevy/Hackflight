#pragma once

#include <console.h>
#include <task.hpp>

class ReceiverTask : public FreeRTOSTask {

    public:


        void begin(void)
        {
            pinMode(LED_BUILTIN, OUTPUT);

            FreeRTOSTask::begin(run, "receiever", this, 4);
        }

        void getRawChannelValues(int16_t chanvals[6])
        {
            static int16_t throttle;
            static int8_t dir;

            dir = 
                dir == 0 ? +1 : 
                throttle == 2000 ? -1 :
                throttle == 1000 ? +1 :
                dir;

            throttle = throttle == 0 ? 1000 : throttle + dir;

            chanvals[0] = throttle;
            chanvals[1] = 1500;
            chanvals[2] = 1500;
            chanvals[3] = 1500;
            chanvals[4] = 1500;
            chanvals[5] = 1500;
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

                    digitalWriteFast(LED_BUILTIN, toggle);

                    toggle = !toggle;

                    prev = msec;
                }

                vTaskDelay(1);

            }
        }
};
