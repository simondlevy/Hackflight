#pragma once

#define BARO_TAB_SIZE_MAX   48

#include "baro.h"

void initSensors(uint16_t * acc_1G, sensor_t * gyro, bool * baro_available, bool * sonar_available);
void ACC_getADC(uint16_t acc_1G);
void Gyro_getADC(sensor_t * gyro);
bool initBaro(baro_t * baro);
void Baro_update(uint32_t * baroPressureSum);
bool initSonar();
void Sonar_update(int32_t * SonarAlt);


