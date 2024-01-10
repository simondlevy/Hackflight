#pragma once

#include <tasks/free_rtos.hpp>

class VisualizerTask : public FreeRTOSTask {

    public:

        void init(void)
        {
            FreeRTOSTask::init(runVisualizerTask, "VISUALIZER", this, 2);

            pinMode(LED_BUILTIN, OUTPUT);
        }

    private:

        static void runVisualizerTask(void * obj)
        {
            ((VisualizerTask *)obj)->run();
        }

        void run(void)
        {
            while (true) {

                Serial.println("BADDA BOOM");
                digitalWriteFast(LED_BUILTIN, LOW);
                vTaskDelay(pdMS_TO_TICKS(500));

                Serial.println("BADDA BING");
                digitalWriteFast(LED_BUILTIN, HIGH);
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        }

};
