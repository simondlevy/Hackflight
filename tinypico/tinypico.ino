#include <TinyPICO.h>

#include <free_rtos_include.h>

static const auto STACKSIZE = 3 * configMINIMAL_STACK_SIZE; // arbitrary

StackType_t taskStackBuffer[STACKSIZE];

StaticTask_t taskBuffer;

static TinyPICO tinypico = TinyPICO();

static constexpr float HEARTBEAT_HZ = 1;
static constexpr uint32_t PULSE_MSEC = 50;

static void device_set(const bool on)
{
    tinypico.DotStar_SetPixelColor(on ? 255 : 0, 0, 0 );
}

static void blink(TickType_t & lastWakeTime, const float rate)
{
    device_set(true);
    vTaskDelay(PULSE_MSEC);
    device_set(false);
    vTaskDelayUntil(&lastWakeTime, 1000/rate);
}


static void fun(void *) 
{
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (true) {

        blink(lastWakeTime, HEARTBEAT_HZ);
    }
}

void setup() 
{
    xTaskCreateStatic(
            fun,      
            "StaticTask",              
            STACKSIZE,    
            NULL, // argument
            1,    // priority 
            taskStackBuffer,         
            &taskBuffer         
            );
}

void loop()
{
}
