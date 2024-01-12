#include <FreeRTOS_TEENSY4.h>
#include <task.h>

#include <tasks/free_rtos.hpp>

class CoreTask : public FreeRTOSTask {

    public:

        void begin(void)
        {
            FreeRTOSTask::begin(run, "CORE", this, 4);
        }

    private:

        static void run(void * obj)
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
