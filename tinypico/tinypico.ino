#include <TinyPICO.h>

// #define STACKSIZE 2048
static const auto STACKSIZE = 3 * configMINIMAL_STACK_SIZE; // arbitrary

StaticTask_t xStaticTaskBuffer;

StackType_t uxStaticTaskStack[STACKSIZE];

static TinyPICO tinypico = TinyPICO();

static void device_set(const bool on)
{
    tinypico.DotStar_SetPixelColor(on ? 255 : 0, 0, 0 );
}

static void fun(void *) 
{
    while (true) {

        device_set(true);
        vTaskDelay(pdMS_TO_TICKS(20));
        device_set(false);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void setup() 
{

    xTaskCreateStatic(
        fun,      
        "StaticTask",              
        STACKSIZE,    
        NULL,                      
        tskIDLE_PRIORITY + 1,      
        uxStaticTaskStack,         
        &xStaticTaskBuffer         
    );
}

void loop()
{
}
