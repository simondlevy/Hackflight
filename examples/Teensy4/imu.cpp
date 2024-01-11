#include <stdint.h>

#include <tasks/free_rtos/imu.hpp>

uint16_t ImuTask::readGyro(Axis3i16* dataOut)
{
    (void)dataOut;
    return 0;
}

void ImuTask::readAccel(Axis3i16* dataOut)
{
    (void)dataOut;
}

void ImuTask::readBaro(void)
{
}
        
void ImuTask::deviceInit(void)
{
}

void ImuTask::interruptInit(void)
{
}
