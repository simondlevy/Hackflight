#include <Arduino.h>

#include <motor.h>

void motorCheckDshotBitbangStatus(void)
{
}

bool motorCheckDshotReady(uint32_t currentTime, uint8_t * tryingToArm)
{
    (void)currentTime;
    (void)tryingToArm;
    return false;
}

float motorConvertFromExternal(uint16_t externalValue)
{
    (void)externalValue;
    return 0;
}

bool motorDshotStreamingCommandsAreEnabled(void)
{
    return false;
}


bool motorIsProtocolDshot(void)
{
    return false;
}

bool motorIsProtocolEnabled(void)
{
    return false;
}

void motorStop(void)
{
}
