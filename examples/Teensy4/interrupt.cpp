#include <stdbool.h>

#include <imxrt.h>

// https://web.eece.maine.edu/~zhu/book/Core/core_cm3_constant.s
#define SCB_ICSR_VECTACTIVE_Pos  0                                   
#define SCB_ICSR_VECTACTIVE_Msk  (0x1FF << SCB_ICSR_VECTACTIVE_Pos)

bool hal_isInInterrupt(void)
{
    return (SCB_ICSR  & SCB_ICSR_VECTACTIVE_Msk) != 0;
}
