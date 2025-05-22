#include <stdbool.h>

#include <stm32fxxx.h>

void resetInterrupts(void)
{
    NVIC_SystemReset();
}
