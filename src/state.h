#pragma once

void stateInit(uint16_t acc_1G);

void stateEstimateAltitude(
        int16_t * angle, 
        int32_t * SonarAlt, 
        int32_t * AltPID, 
        int32_t * EstAlt, 
        int32_t * AltHold, 
        int32_t * setVelocity, 
        int32_t * errorVelocityI, 
        int32_t * vario, 
        bool      velocityControl, 
        uint32_t  baroPressureSum, 
        bool      armed);

bool stateEstimateAttitude(
        sensor_t * acc, 
        sensor_t * gyro, 
        int16_t *  accSmooth,
        int16_t *  gyroData,
        int16_t *  angle,
        int16_t *  heading, 
        int16_t *  throttleAngleCorrection, 
        bool       armed);
