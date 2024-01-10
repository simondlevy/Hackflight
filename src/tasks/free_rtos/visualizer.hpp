#pragma once

class VisualizerTask {

    public:

        void init(void)
        {
            xTaskCreate(task1, "task1", 128, nullptr, 2, nullptr);

            pinMode(LED_BUILTIN, OUTPUT);
        }

    private:

        static void task1(void*) 
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
