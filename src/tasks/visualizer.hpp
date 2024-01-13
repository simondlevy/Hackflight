#pragma once

#include <console.h>
#include <task.hpp>
#include <tasks/receiver.hpp>
#include <tasks/guestimator.hpp>
#include <visualizer.hpp>

class VisualizerTask : public FreeRTOSTask {

    public:

        void begin(EstimatorTask * estimatorTask, ReceiverTask * receiverTask)
        {
            _receiverTask = receiverTask;
            _estimatorTask = estimatorTask;

            FreeRTOSTask::begin(run, "visualizer", this, 2);
        }

    private:

        static void run(void * obj) 
        {
            ((VisualizerTask *)obj)->run();
        }

        Visualizer _visualizer;

        EstimatorTask * _estimatorTask;

        ReceiverTask * _receiverTask;

        void run(void)
        {
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
