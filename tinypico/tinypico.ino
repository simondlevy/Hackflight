/*
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"*/

#include <TinyPICO.h>

#define STATIC_TASK_STACK_SIZE 2048

StaticTask_t xStaticTaskBuffer;
StackType_t uxStaticTaskStack[STATIC_TASK_STACK_SIZE];

static TinyPICO tinypico = TinyPICO();

static void device_set(const bool on)
{
    tinypico.DotStar_SetPixelColor(on ? 255 : 0, 0, 0 );
}


static void static_task_function(void *pvParameters) 
{
    for (;;) {

        device_set(true);
        vTaskDelay(pdMS_TO_TICKS(20));
        device_set(false);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void setup() 
{

    xTaskCreateStatic(
        static_task_function,      
        "StaticTask",              
        STATIC_TASK_STACK_SIZE,    
        NULL,                      
        tskIDLE_PRIORITY + 1,      
        uxStaticTaskStack,         
        &xStaticTaskBuffer         
    );
}

void loop()
{
}
