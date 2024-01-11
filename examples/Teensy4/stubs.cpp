#include <stdint.h>

#include <crossplatform.h>
#include <datatypes.h>
#include <tasks/free_rtos/imu.hpp>

void systemWaitStart(void)
{
}

bool hal_isInInterrupt(void)
{
    return false;
}

void motorsCheckDshot()
{
}

int  motorsGetRatio(uint32_t id)
{
    (void)id;
    return 0;
}

void motorsInit(void)
{
}

bool motorsTest(void)
{
    return false;
}

void motorsSetRatios(const uint16_t ratios[])
{
    (void)ratios;
}

extern "C" {
    void  motorsStop()
    {
    }

}
        
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
