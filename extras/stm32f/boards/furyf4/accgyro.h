/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "platform.h"

#include "drivers/exti.h"
#include "drivers/bus.h"
#include "drivers/sensor.h"

typedef void (*mpuResetFnPtr)(void);

typedef struct mpuConfiguration_s {
    mpuResetFnPtr resetFn;
} mpuConfiguration_t;

typedef enum {
    MPU_NONE,
    MPU_3050,
    MPU_60x0,
    MPU_60x0_SPI,
    MPU_65xx_I2C,
    MPU_65xx_SPI,
    MPU_9250_SPI,
    ICM_20601_SPI,
    ICM_20602_SPI,
    ICM_20608_SPI,
    ICM_20649_SPI,
    ICM_20689_SPI,
    BMI_160_SPI,
} mpuSensor_e;

typedef enum {
    MPU_HALF_RESOLUTION,
    MPU_FULL_RESOLUTION
} mpu6050Resolution_e;

typedef struct mpuDetectionResult_s {
    mpuSensor_e sensor;
    mpu6050Resolution_e resolution;
} mpuDetectionResult_t;
#include "sensors/gyro.h"

#define MPU_I2C_INSTANCE I2C_DEVICE
#define GYRO_HARDWARE_LPF_NORMAL       0
#define GYRO_HARDWARE_LPF_EXPERIMENTAL 1
#define GYRO_HARDWARE_LPF_1KHZ_SAMPLE  2

#define GYRO_32KHZ_HARDWARE_LPF_NORMAL       0
#define GYRO_32KHZ_HARDWARE_LPF_EXPERIMENTAL 1

typedef enum {
    GYRO_RATE_1_kHz,
    GYRO_RATE_1100_Hz,
    GYRO_RATE_3200_Hz,
    GYRO_RATE_8_kHz,
    GYRO_RATE_9_kHz,
    GYRO_RATE_32_kHz,
} gyroRateKHz_e;

typedef struct gyroDev_s {
    sensorGyroInitFuncPtr initFn;                             // initialize function
    sensorGyroReadFuncPtr readFn;                             // read 3 axis data function
    sensorGyroReadDataFuncPtr temperatureFn;                  // read temperature if available
    extiCallbackRec_t exti;
    busDevice_t bus;
    float scale;                                            // scalefactor
    float gyroZero[XYZ_AXIS_COUNT];
    float gyroADC[XYZ_AXIS_COUNT];                        // gyro data after calibration and alignment
    float gyroADCf[XYZ_AXIS_COUNT];
    int32_t gyroADCRawPrevious[XYZ_AXIS_COUNT];
    int16_t gyroADCRaw[XYZ_AXIS_COUNT];
    int16_t temperature;
    mpuConfiguration_t mpuConfiguration;
    mpuDetectionResult_t mpuDetectionResult;
    sensor_align_e gyroAlign;
    gyroRateKHz_e gyroRateKHz;
    bool dataReady;
    bool gyro_high_fsr;
    uint8_t hardware_lpf;
    uint8_t hardware_32khz_lpf;
    uint8_t mpuDividerDrops;
    ioTag_t mpuIntExtiTag;
    uint8_t gyroHasOverflowProtection;
    gyroSensor_e gyroHardware;
} gyroDev_t;

typedef struct accDev_s {
    sensorAccInitFuncPtr initFn;                              // initialize function
    sensorAccReadFuncPtr readFn;                              // read 3 axis data function
    busDevice_t bus;
    uint16_t acc_1G;
    int16_t ADCRaw[XYZ_AXIS_COUNT];
    mpuDetectionResult_t mpuDetectionResult;
    sensor_align_e accAlign;
    bool dataReady;
    bool acc_high_fsr;
    char revisionCode;                                      // a revision code for the sensor, if known
    uint8_t filler[2];
} accDev_t;
