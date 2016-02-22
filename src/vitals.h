#pragma once

#define RC_CHANS 8

typedef struct {

    int16_t  accSmooth[3];
    uint16_t acc1G;
    int16_t  imuAngle[2];
    bool     armed; 
    uint32_t baroPressureSum; 
    uint16_t cycleTime; 
    int32_t  estAlt; 
    int16_t  gyroData[3];
    int16_t  heading; 
    int16_t  magADC[3];
    int16_t  motorDisarmed[4];
    int16_t  motors[4];
    uint16_t rcData[RC_CHANS]; 
    int32_t  sonarAlt; 
    int32_t  vario; 

} vitals_t;
