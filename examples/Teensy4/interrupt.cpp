#include <stdint.h>

#include <crossplatform.h>
#include <datatypes.h>
#include <tasks/free_rtos/imu.hpp>

bool hal_isInInterrupt(void)
{
    return false;
}
