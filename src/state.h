#pragma once

void stateInit(uint16_t acc1G);

void stateEstimateAltitude(vitals_t * vitals, int32_t * AltPID, int32_t * AltHold, int32_t * setVelocity, int32_t * errorVelocityI, bool velocityControl);

bool stateEstimateAttitude(vitals_t * vitals, sensor_t * acc, sensor_t * gyro, int16_t *  throttleAngleCorrection);
