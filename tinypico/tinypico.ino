#include <hackflight.h>
#include <tasks/led.hpp>

static LedTask ledTask;

void setup() 
{
    ledTask.begin();
}

void loop()
{
}
