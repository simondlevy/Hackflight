#include <Arduino.h>
#include <Wire.h>
#include <USFSMAX.h>

#include <imu.h>


void imuGetQuaternion(hackflight_t * hf, uint32_t time, quaternion_t * quat)
{
    (void)hf;
    (void)time;
    (void)quat;
}

void imuInit(void)
{
}

void imuUpdateFusion(hackflight_t * hf, uint32_t time, quaternion_t * quat, rotation_t * rot)
{
    (void)hf;
    (void)time;
    (void)quat;
    (void)rot;
}
