#include <TinyPICO.h>

#include <free_rtos_include.h>

class LedTask {

    public:

        void begin()
        {
            xTaskCreateStatic(
                    fun,      
                    "StaticTask",              
                    STACKSIZE,    
                    this, // argument
                    2,    // priority 
                    _taskStackBuffer,         
                    &_taskBuffer         
                    );        
        }

    private:

        static const auto STACKSIZE = 3 * configMINIMAL_STACK_SIZE; // arbitrary

        static constexpr float HEARTBEAT_HZ = 1;
        static constexpr uint32_t PULSE_MSEC = 50;

        StackType_t _taskStackBuffer[STACKSIZE];

        StaticTask_t _taskBuffer;

        TinyPICO tinypico = TinyPICO();

        static void fun(void * arg) 
        {
            TickType_t lastWakeTime = xTaskGetTickCount();

            LedTask * ledTask = (LedTask *)arg;

            while (true) {

                ledTask->blink(lastWakeTime, HEARTBEAT_HZ);
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
