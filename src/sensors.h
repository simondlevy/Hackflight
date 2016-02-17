
#pragma once

#include "baro.h"

bool sonar_available;

void     ACC_getADC(uint16_t acc_1G);
int      Baro_update(uint32_t currentTime);
void     Gyro_getADC(void);
bool     initBaro(baro_t * baro);
uint16_t initSensors(int hwrev);
bool     initSonar();
void     Sonar_update(void);
