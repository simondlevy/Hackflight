#include <stdbool.h>

#include <stm32fxxx.h>

bool hal_isInInterrupt(void)
{
    return (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;
}

bool isDebuggingOn(void)
{
  return (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0;
}

void resetInterrupts(void)
{
    NVIC_SystemReset();
}
