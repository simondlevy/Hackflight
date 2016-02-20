#pragma once

void mspInit(void);

void mspCom(
        int16_t * angle,
        int16_t * gyroData,
        uint16_t * rcData, 
        int16_t * accSmooth,
        int16_t * magADC,
        int32_t SonarAlt, 
        int32_t EstAlt, 
        int32_t vario, 
        int16_t heading, 
        int16_t * motors,
        int16_t * motor_disarmed,
        uint32_t baroPressureSum, 
        uint16_t cycleTime, 
        bool armed, 
        uint16_t acc_1G);
