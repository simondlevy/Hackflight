#pragma once

#define BARO_TAB_SIZE_MAX   48

#include "baro.h"
void initSensors(sensor_t * acc, sensor_t * gyro, baro_t * baro, uint16_t * acc_1G, bool * baro_available, bool * sonar_available);
void ACC_getADC(sensor_t * acc, int16_t * accADC);
void Gyro_getADC(sensor_t * gyro, int16_t * gyroADC);
bool initBaro(baro_t * baro);
void Baro_update(baro_t * baro, uint32_t * baroPressureSum);
bool initSonar();
void Sonar_update(int32_t * SonarAlt);
void sensorsInitAccelCalibration(void);
void sensorsInitGyroCalibration(void);
bool sensorsCalibratingA(void);
bool sensorsCalibratingG(void);
