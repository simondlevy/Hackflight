
#pragma once

#include "baro.h"

uint16_t  ACC_getADC(uint16_t calibratingA, uint16_t acc_1G);
int       Baro_update(uint32_t currentTime);
uint16_t  Gyro_getADC(uint16_t calibratingG);
bool      initBaro(baro_t * baro);
void      initSensors(int hwrev, uint16_t * acc_1G, bool * baro_available, bool * sonar_available);
bool      initSonar();
void      Sonar_update(void);
