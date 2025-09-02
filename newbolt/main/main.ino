#include <STM32FreeRTOS.h>

#include "led.hpp"
#include "safety.hpp"

static const uint8_t LED_PIN = PC0;

static Motors motors;

static Safety safety = Safety(&motors);

static LedTask ledTask;

void setup() 
{
    Serial.begin(115200);

    ledTask.begin(&safety, LED_PIN, true);

    vTaskStartScheduler();

    Serial.println("Insufficient RAM");

    while (true);
}

void loop() 
{
}
