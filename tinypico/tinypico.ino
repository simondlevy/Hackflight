#include <TinyPICO.h>

static TaskHandle_t Task1;

static TinyPICO tinypico = TinyPICO();

static void device_set(const bool on)
{
    tinypico.DotStar_SetPixelColor(on ? 255 : 0, 0, 0 );
}

static void Task1code( void * pvParameters )
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
    xTaskCreatePinnedToCore(
            Task1code,   
            "Task1",     
            10000,       // stack size 
            NULL,        // parameter
            1,           // priority
            &Task1,      
            0);          // core
}

void loop() 
{
}
