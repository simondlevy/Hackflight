#include <Arduino.h>

#include <system.h>
#include <time.h>
 
uint32_t systemGetCycleCounter(void)
{
    return ARM_DWT_CYCCNT;
}
