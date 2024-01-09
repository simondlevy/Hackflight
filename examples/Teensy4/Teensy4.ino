#include <FreeRTOS_TEENSY4.h>
#include <task.h>

#include <vl53l1_arduino.h>

#include <hfheader.h>
#include <tasks/free_rtos/core.hpp>
#include <tasks/free_rtos/visualizer.hpp>

void setup() 
{

    static VL53L1_Arduino vl53l1;    

    static CoreTask coreTask;

    Wire.begin();

    vl53l1.begin();

    Serial.begin(115200);

    // vTaskStartScheduler();
}

void loop(void) 
{
}
