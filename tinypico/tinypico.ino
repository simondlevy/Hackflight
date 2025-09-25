#include <free_rtos_include.h>

#include <TinyPICO.h>

static const auto STACKSIZE = 3 * configMINIMAL_STACK_SIZE; // arbitrary

static TaskHandle_t Task1;

static TinyPICO tinypico = TinyPICO();

static StackType_t  taskStackBuffer[STACKSIZE]; 

static StaticTask_t taskTaskBuffer;


static void device_set(const bool on)
{
    tinypico.DotStar_SetPixelColor(on ? 255 : 0, 0, 0 );
}

static void fun(void * arg)
{
    while (true) {

        device_set(true);
        delay(20);
        device_set(false);
        delay(1000);
    } 
}

void setup() 
{
    xTaskCreate(
            fun,   
            "Task1",     
            10000,       // stack size 
            NULL,        // parameter
            1,           // priority
            &Task1);      
}

void loop() 
{
}
