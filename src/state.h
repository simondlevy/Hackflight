#pragma once

void stateInit(uint16_t acc1G);

void stateEstimateAltitude(vitals_t * vitals, int32_t * AltPID, int32_t * AltHold, int32_t * setVelocity, int32_t * errorVelocityI, bool velocityControl);

bool stateEstimateAttitude(
        sensor_t * acc, 
        sensor_t * gyro, 
        int16_t *  accSmooth,
        int16_t *  gyroData,
        int16_t *  angle,
        int16_t *  heading, 
        int16_t *  throttleAngleCorrection, 
        bool       armed);
