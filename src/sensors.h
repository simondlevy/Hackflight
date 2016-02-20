#pragma once

#define BARO_TAB_SIZE_MAX   48

#include "baro.h"
void sensorsInit(
        sensor_t * acc, 
        sensor_t * gyro, 
        baro_t * baro, 
        uint16_t * acc_1G, 
        bool * baro_available, 
        bool * sonar_available);

void sensorsGetAccel(sensor_t * acc, int16_t * accADC);

void sensorsGetGyro(sensor_t * gyro, int16_t * gyroADC);

void sensorsUpdateBaro(baro_t * baro, uint32_t * baroPressureSum);

void sensorsUpdateSonar(int32_t * SonarAlt);

void sensorsInitAccelCalibration(void);

void sensorsInitGyroCalibration(void);

bool sensorsCalibratingA(void);

bool sensorsCalibratingG(void);

bool initSonar();

