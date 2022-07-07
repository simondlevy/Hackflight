/*
This file is part of Hackflight.

Hackflight is free software: you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
Hackflight. If not, see <https://www.gnu.org/licenses/>.
*/

#include <stdbool.h>
#include <stdint.h>

#include <maths.h>
#include <system.h>
#include <time.h>

#include "platform.h"
#include "imu_mpu.h"
#include "bus_spi.h"
#include "exti.h"
#include "io.h"
#include "systemdev.h"

// RF = Register Flag
static const uint8_t MPU_RF_DATA_RDY_EN = 0x01;

static const uint8_t MPU6000_CONFIG              = 0x1A;

// 1 MHz max SPI frequency for initialisation
static const uint32_t MPU6000_MAX_SPI_INIT_CLK_HZ = 1000000;

// 20 MHz max SPI frequency
static const uint32_t MPU6000_MAX_SPI_CLK_HZ = 20000000;

static uint32_t MPU6000_SHORT_THRESHOLD = 82;

static const uint8_t MPU_CLK_SEL_PLLGYROZ = 0x03;

static const uint8_t BIT_TEMP       = 0x01;
static const uint8_t BIT_ACC        = 0x02;
static const uint8_t BIT_GYRO       = 0x04;
static const uint8_t BIT_I2C_IF_DIS = 0x10;
static const uint8_t BIT_H_RESET    = 0x80;

static void mpu6000AccAndGyroInit(gyroDev_t *gyro)
{
    spiSetClkDivisor(&gyro->dev, spiCalculateDivider(MPU6000_MAX_SPI_INIT_CLK_HZ));

    // Device was already reset during detection so proceed with configuration

    // Clock Source PPL with Z axis gyro reference
    spiWriteReg(&gyro->dev, MPU_RA_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
    delayMicroseconds(15);

    // Disable Primary I2C Interface
    spiWriteReg(&gyro->dev, MPU_RA_USER_CTRL, BIT_I2C_IF_DIS);
    delayMicroseconds(15);

    spiWriteReg(&gyro->dev, MPU_RA_PWR_MGMT_2, 0x00);
    delayMicroseconds(15);

    // Accel Sample Rate 1kHz
    // Gyroscope Output Rate =  1kHz when the DLPF is enabled
    spiWriteReg(&gyro->dev, MPU_RA_SMPLRT_DIV, 0);
    delayMicroseconds(15);

    // Gyro +/- 2000 DPS Full Scale
    spiWriteReg(&gyro->dev, MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3);
    delayMicroseconds(15);

    // Accel +/- 16 G Full Scale
    spiWriteReg(&gyro->dev, MPU_RA_ACCEL_CONFIG, INV_FSR_16G << 3);
    delayMicroseconds(15);

    // INT_ANYRD_2CLEAR
    spiWriteReg(
            &gyro->dev,
            MPU_RA_INT_PIN_CFG,
            0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0);  

    delayMicroseconds(15);

    spiWriteReg(&gyro->dev, MPU_RA_INT_ENABLE, MPU_RF_DATA_RDY_EN);
    delayMicroseconds(15);

    spiSetClkDivisor(&gyro->dev, spiCalculateDivider(MPU6000_MAX_SPI_CLK_HZ));
    delayMicroseconds(1);
}


void mpu6000SpiGyroInit(gyroDev_t *gyro)
{
    mpuGyroInit(gyro);

    mpu6000AccAndGyroInit(gyro);

    spiSetClkDivisor(&gyro->dev, spiCalculateDivider(MPU6000_MAX_SPI_INIT_CLK_HZ));

    // Accel and Gyro DLPF Setting
    spiWriteReg(&gyro->dev, MPU6000_CONFIG, mpuGyroDLPF(gyro));
    delayMicroseconds(1);

    spiSetClkDivisor(&gyro->dev, spiCalculateDivider(MPU6000_MAX_SPI_CLK_HZ));

    if (((int8_t)gyro->adcRaw[1]) == -1 && ((int8_t)gyro->adcRaw[0]) == -1) {
        systemFailureMode(FAILURE_GYRO_INIT_FAILED);
    }
}

// ----------------------------------------------------------------------------

uint8_t mpuBusDetect(const extDevice_t *dev)
{

    spiSetClkDivisor(dev, spiCalculateDivider(MPU6000_MAX_SPI_INIT_CLK_HZ));

    // reset the device configuration
    spiWriteReg(dev, MPU_RA_PWR_MGMT_1, BIT_H_RESET);
    delay(100);  // datasheet specifies a 100ms delay after reset

    // reset the device signal paths
    spiWriteReg(dev, MPU_RA_SIGNAL_PATH_RESET, BIT_GYRO | BIT_ACC | BIT_TEMP);
    delay(100);  // datasheet specifies a 100ms delay after signal path reset


    const uint8_t whoAmI = spiReadRegMsk(dev, MPU_RA_WHO_AM_I);
    delayMicroseconds(1); // Ensure CS high time is met which is violated on H7 without this delay
    uint8_t detectedSensor = MPU_NONE;

    if (whoAmI == MPU6000_WHO_AM_I_CONST) {
        detectedSensor = MPU_60x0_SPI;
    }

    spiSetClkDivisor(dev, spiCalculateDivider(MPU6000_MAX_SPI_CLK_HZ));
    return detectedSensor;
}

bool mpuBusAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != MPU_60x0_SPI) {
        return false;
    }

    acc->readFn = mpuAccReadSPI;

    return true;
}

bool mpuBusGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != MPU_60x0_SPI) {
        return false;
    }

    gyro->initFn = mpu6000SpiGyroInit;
    gyro->readFn = mpuGyroReadSPI;
    gyro->scaleDps = 2000;
    gyro->gyroShortPeriod = systemClockMicrosToCycles(MPU6000_SHORT_THRESHOLD);
    return true;
}
