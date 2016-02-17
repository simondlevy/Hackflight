
#pragma once

#include "baro.h"

bool sonar_available;

void initSensors(int hwrev);
void ACC_getADC(void);
void Gyro_getADC(void);
bool initBaro(baro_t * baro);
int Baro_update(uint32_t currentTime);
bool initSonar();
void Sonar_update(void);
