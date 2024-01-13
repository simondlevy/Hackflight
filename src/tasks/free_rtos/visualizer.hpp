#pragma once

#include <console.h>
#include <tasks/free_rtos.hpp>
#include <visualizer.hpp>

class VisualizerTask : public FreeRTOSTask {

    public:


        void begin(void)
        {
            FreeRTOSTask::begin(run, "VISUALIZER", this, 2);
        }

    private:

        static void run(void * obj) 
        {
            ((VisualizerTask *)obj)->run();
        }

        Visualizer _visualizer;

        void run(void)
        {
            consolePrintf("CORE: Starting loop\n");

            while (true) {

                while (Serial.available()) {

                    if (_visualizer.parse(Serial.read())) {

                        while (_visualizer.available()) {

                            Serial.write(_visualizer.read());
                        }
                    }
                }

            }
        }
};
