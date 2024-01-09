#include <FreeRTOS_TEENSY4.h>

#include <vl53l1_arduino.h>

#include "tasks.h"

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
