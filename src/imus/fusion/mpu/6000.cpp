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

#include <constrain.h>
#include <imus/fusion/mpu/6000.h>
#include <system.h>
#include <time.h>

#include <platform.h>
#include <bus_spi.h>
#include <exti.h>
#include <io.h>
#include <systemdev.h>

void Mpu6000Imu::mpuBusInit(gyroDev_t *gyro)
{
    MpuImu::gyroInit();

    spiSetClkDivisor(&gyro->dev, spiCalculateDivider(Mpu6000Imu::MAX_SPI_INIT_CLK_HZ));

    // Device was already reset during detection so proceed with configuration

    // Clock Source PPL with Z axis gyro reference
    spiWriteReg(&gyro->dev, MpuImu::RA_PWR_MGMT_1, Mpu6000Imu::CLK_SEL_PLLGYROZ);
    delayMicroseconds(15);

    // Disable Primary I2C Interface
    spiWriteReg(&gyro->dev, MpuImu::RA_USER_CTRL, Mpu6000Imu::BIT_I2C_IF_DIS);
    delayMicroseconds(15);

    spiWriteReg(&gyro->dev, MpuImu::RA_PWR_MGMT_2, 0x00);
    delayMicroseconds(15);

    // Accel Sample Rate 1kHz
    // Gyroscope Output Rate =  1kHz when the DLPF is enabled
    spiWriteReg(&gyro->dev, MpuImu::RA_SMPLRT_DIV, 0);
    delayMicroseconds(15);

    // Gyro +/- 2000 DPS Full Scale
    spiWriteReg(&gyro->dev, MpuImu::RA_GYRO_CONFIG, MpuImu::INV_FSR_2000DPS << 3);
    delayMicroseconds(15);

    // Accel +/- 16 G Full Scale
    spiWriteReg(&gyro->dev, MpuImu::RA_ACCEL_CONFIG, MpuImu::INV_FSR_16G << 3);
    delayMicroseconds(15);

    spiWriteReg(&gyro->dev, MpuImu::RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0);  // INT_ANYRD_2CLEAR
    delayMicroseconds(15);

    spiWriteReg(&gyro->dev, MpuImu::RA_INT_ENABLE, Mpu6000Imu::RF_DATA_RDY_EN);
    delayMicroseconds(15);

    spiSetClkDivisor(&gyro->dev, spiCalculateDivider(Mpu6000Imu::MAX_SPI_CLK_HZ));
    delayMicroseconds(1);

    spiSetClkDivisor(&gyro->dev, spiCalculateDivider(Mpu6000Imu::MAX_SPI_INIT_CLK_HZ));

    // Accel and Gyro DLPF Setting
    spiWriteReg(&gyro->dev, Mpu6000Imu::CONFIG, 0); // no gyro DLPF
    delayMicroseconds(1);

    spiSetClkDivisor(&gyro->dev, spiCalculateDivider(Mpu6000Imu::MAX_SPI_CLK_HZ));

    MpuImu::gyroRead();

    if (((int8_t)gyro->adcRaw[1]) == -1 && ((int8_t)gyro->adcRaw[0]) == -1) {
        systemFailureMode(FAILURE_GYRO_INIT_FAILED);
    }
}

// ----------------------------------------------------------------------------

mpuSensor_e mpuBusDetect(const extDevice_t *dev)
{

    spiSetClkDivisor(dev, spiCalculateDivider(Mpu6000Imu::MAX_SPI_INIT_CLK_HZ));

    // reset the device configuration
    spiWriteReg(dev, MpuImu::RA_PWR_MGMT_1, Mpu6000Imu::BIT_H_RESET);
    delay(100);  // datasheet specifies a 100ms delay after reset

    // reset the device signal paths
    spiWriteReg(dev, MpuImu::RA_SIGNAL_PATH_RESET,
            Mpu6000Imu::BIT_GYRO | Mpu6000Imu::BIT_ACC | Mpu6000Imu::BIT_TEMP);
    delay(100);  // datasheet specifies a 100ms delay after signal path reset


    const uint8_t whoAmI = spiReadRegMsk(dev, MpuImu::RA_WHO_AM_I);

    // Ensure CS high time is met which is violated on H7 without this delay
    delayMicroseconds(1); 
    mpuSensor_e detectedSensor = MPU_NONE;

    if (whoAmI == MpuImu::MPU6000_WHO_AM_I_CONST) {
        const uint8_t productID = spiReadRegMsk(dev, MpuImu::RA_PRODUCT_ID);

        // verify product revision
        switch (productID) {
            case Mpu6000Imu::ES_REV_C4:
            case Mpu6000Imu::ES_REV_C5:
            case Mpu6000Imu::REV_C4:
            case Mpu6000Imu::REV_C5:
            case Mpu6000Imu::ES_REV_D6:
            case Mpu6000Imu::ES_REV_D7:
            case Mpu6000Imu::ES_REV_D8:
            case Mpu6000Imu::REV_D6:
            case Mpu6000Imu::REV_D7:
            case Mpu6000Imu::REV_D8:
            case Mpu6000Imu::REV_D9:
            case Mpu6000Imu::REV_D10:
                detectedSensor = MPU_60x0_SPI;
        }
    }

    spiSetClkDivisor(dev, spiCalculateDivider(Mpu6000Imu::MAX_SPI_CLK_HZ));
    return detectedSensor;
}

bool mpuBusGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != MPU_60x0_SPI) {
        return false;
    }

    gyro->readFn = MpuImu::gyroReadSPI;
    gyro->gyroShortPeriod = systemClockMicrosToCycles(Mpu6000Imu::SHORT_THRESHOLD);
    return true;
}
