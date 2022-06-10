#include <Arduino.h>

#include <system.h>
#include <time.h>
 
void delayMillis(uint32_t msec)
{
    delay(msec);
}

uint32_t systemClockMicrosToCycles(uint32_t usec)
{
    (void)usec;
    return 0;
}

uint32_t systemGetCycleCounter(void)
{
    return 0;
}

void systemReboot(void)
{
}

uint32_t timeMicros(void)
{
    return micros();
}

uint32_t timeMillis(void)
{
    return millis();
}
