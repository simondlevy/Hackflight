/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */
#pragma once

bool mpu6050_init(bool cuttingEdge, sensor_t *acc, sensor_t *gyro, uint8_t lpf);
