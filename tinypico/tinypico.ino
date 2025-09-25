#include <TinyPICO.h>

#include <free_rtos_include.h>

static const auto STACKSIZE = 3 * configMINIMAL_STACK_SIZE; // arbitrary

static constexpr float HEARTBEAT_HZ = 1;
static constexpr uint32_t PULSE_MSEC = 50;


StaticTask_t xStaticTaskBuffer;

StackType_t uxStaticTaskStack[STACKSIZE];

static TinyPICO tinypico = TinyPICO();

static void device_set(const bool on)
{
    tinypico.DotStar_SetPixelColor(on ? 255 : 0, 0, 0 );
}

void blink(const float rate)
{
    device_set(true);
    vTaskDelay(PULSE_MSEC);
    device_set(false);
    vTaskDelay(1000/rate);
}


static void fun(void *) 
{
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (true) {

        blink(HEARTBEAT_HZ);

        /*
        device_set(true);
        vTaskDelay(20);
        device_set(false);
        vTaskDelay(1000);*/
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
            uxStaticTaskStack,         
            &xStaticTaskBuffer         
            );
}

void loop()
{
}
