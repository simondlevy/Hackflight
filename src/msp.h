#pragma once

typedef struct {

    int16_t *  accSmooth;
    uint16_t   acc1G;
    int16_t *  angle;
    bool       armed; 
    uint32_t   baroPressureSum; 
    uint16_t   cycleTime; 
    int32_t    estAlt; 
    int16_t *  gyroData;
    int16_t    heading; 
    int16_t *  magADC;
    int16_t *  motorDisarmed;
    int16_t *  motors;
    uint16_t * rcData; 
    int32_t    sonarAlt; 
    int32_t    vario; 

} telemetry_t;

void mspInit(void);

void mspCom(telemetry_t * telemetry);

