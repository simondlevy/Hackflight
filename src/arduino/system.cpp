#include <Arduino.h>

#include <system.h>
#include <time.h>
 
void delayMillis(const uint32_t msec)
{
    delay(msec);
}

uint32_t systemClockMicrosToCycles(const uint32_t usec)
{
    return microsecondsToClockCycles(usec);
}

void systemReboot(void)
{
    // unsued
}

uint32_t timeMicros(void)
{
    return micros();
}

uint32_t timeMillis(void)
{
    return millis();
}
