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

#pragma once

#include <constrain.h>
#include <imus/fusion.h>
#include <time.h>

#include "mpudev.h"

#include <bus.h>
#include <exti.h>

class MpuImu : public FusionImu {

    public:

        static const uint8_t RA_WHO_AM_I         = 0x75;
        static const uint8_t RA_WHO_AM_I_LEGACY  = 0x00;

        static const uint8_t MPUx0x0_WHO_AM_I_CONST   = 0x68 ;
        static const uint8_t MPU6000_WHO_AM_I_CONST   = 0x68;
        static const uint8_t MPU6500_WHO_AM_I_CONST   = 0x70;
        static const uint8_t MPU9250_WHO_AM_I_CONST   = 0x71;
        static const uint8_t MPU9255_WHO_AM_I_CONST   = 0x73;
        static const uint8_t ICM20601_WHO_AM_I_CONST  = 0xAC;
        static const uint8_t ICM20602_WHO_AM_I_CONST  = 0x12;
        static const uint8_t ICM20608G_WHO_AM_I_CONST = 0xAF;
        static const uint8_t ICM20649_WHO_AM_I_CONST  = 0xE1;
        static const uint8_t ICM20689_WHO_AM_I_CONST  = 0x98;
        static const uint8_t ICM42605_WHO_AM_I_CONST  = 0x42;
        static const uint8_t ICM42688P_WHO_AM_I_CONST = 0x47;

        // RA = Register Address;

        static const uint8_t RA_XG_OFFS_TC         = 0x00;
        static const uint8_t RA_YG_OFFS_TC         = 0x01;
        static const uint8_t RA_ZG_OFFS_TC         = 0x02;
        static const uint8_t RA_X_FINE_GAIN        = 0x03;
        static const uint8_t RA_Y_FINE_GAIN        = 0x04;
        static const uint8_t RA_Z_FINE_GAIN        = 0x05;
        static const uint8_t RA_XA_OFFS_H          = 0x06;
        static const uint8_t RA_XA_OFFS_L_TC       = 0x07;
        static const uint8_t RA_YA_OFFS_H          = 0x08;
        static const uint8_t RA_YA_OFFS_L_TC       = 0x09;
        static const uint8_t RA_ZA_OFFS_H          = 0x0A;
        static const uint8_t RA_ZA_OFFS_L_TC       = 0x0B;
        static const uint8_t RA_PRODUCT_ID         = 0x0C;
        static const uint8_t RA_XG_OFFS_USRH       = 0x13;
        static const uint8_t RA_XG_OFFS_USRL       = 0x14;
        static const uint8_t RA_YG_OFFS_USRH       = 0x15;
        static const uint8_t RA_YG_OFFS_USRL       = 0x16;
        static const uint8_t RA_ZG_OFFS_USRH       = 0x17;
        static const uint8_t RA_ZG_OFFS_USRL       = 0x18;
        static const uint8_t RA_SMPLRT_DIV         = 0x19;
        static const uint8_t RA_CONFIG             = 0x1A;
        static const uint8_t RA_GYRO_CONFIG        = 0x1B;
        static const uint8_t RA_ACCEL_CONFIG       = 0x1C;
        static const uint8_t RA_FF_THR             = 0x1D;
        static const uint8_t RA_FF_DUR             = 0x1E;
        static const uint8_t RA_MOT_THR            = 0x1F;
        static const uint8_t RA_MOT_DUR            = 0x20;
        static const uint8_t RA_ZRMOT_THR          = 0x21;
        static const uint8_t RA_ZRMOT_DUR          = 0x22;
        static const uint8_t RA_FIFO_EN            = 0x23;
        static const uint8_t RA_I2C_MST_CTRL       = 0x24;
        static const uint8_t RA_I2C_SLV0_ADDR      = 0x25;
        static const uint8_t RA_I2C_SLV0_REG       = 0x26;
        static const uint8_t RA_I2C_SLV0_CTRL      = 0x27;
        static const uint8_t RA_I2C_SLV1_ADDR      = 0x28;
        static const uint8_t RA_I2C_SLV1_REG       = 0x29;
        static const uint8_t RA_I2C_SLV1_CTRL      = 0x2A;
        static const uint8_t RA_I2C_SLV2_ADDR      = 0x2B;
        static const uint8_t RA_I2C_SLV2_REG       = 0x2C;
        static const uint8_t RA_I2C_SLV2_CTRL      = 0x2D;
        static const uint8_t RA_I2C_SLV3_ADDR      = 0x2E;
        static const uint8_t RA_I2C_SLV3_REG       = 0x2F;
        static const uint8_t RA_I2C_SLV3_CTRL      = 0x30;
        static const uint8_t RA_I2C_SLV4_ADDR      = 0x31;
        static const uint8_t RA_I2C_SLV4_REG       = 0x32;
        static const uint8_t RA_I2C_SLV4_DO        = 0x33;
        static const uint8_t RA_I2C_SLV4_CTRL      = 0x34;
        static const uint8_t RA_I2C_SLV4_DI        = 0x35;
        static const uint8_t RA_I2C_MST_STATUS     = 0x36;
        static const uint8_t RA_INT_PIN_CFG        = 0x37;
        static const uint8_t RA_INT_ENABLE         = 0x38;
        static const uint8_t RA_DMP_INT_STATUS     = 0x39;
        static const uint8_t RA_INT_STATUS         = 0x3A;
        static const uint8_t RA_ACCEL_XOUT_H       = 0x3B;
        static const uint8_t RA_ACCEL_XOUT_L       = 0x3C;
        static const uint8_t RA_ACCEL_YOUT_H       = 0x3D;
        static const uint8_t RA_ACCEL_YOUT_L       = 0x3E;
        static const uint8_t RA_ACCEL_ZOUT_H       = 0x3F;
        static const uint8_t RA_ACCEL_ZOUT_L       = 0x40;
        static const uint8_t RA_TEMP_OUT_H         = 0x41;
        static const uint8_t RA_TEMP_OUT_L         = 0x42;
        static const uint8_t RA_GYRO_XOUT_H        = 0x43;
        static const uint8_t RA_GYRO_XOUT_L        = 0x44;
        static const uint8_t RA_GYRO_YOUT_H        = 0x45;
        static const uint8_t RA_GYRO_YOUT_L        = 0x46;
        static const uint8_t RA_GYRO_ZOUT_H        = 0x47;
        static const uint8_t RA_GYRO_ZOUT_L        = 0x48;
        static const uint8_t RA_EXT_SENS_DATA_00   = 0x49;
        static const uint8_t RA_MOT_DETECT_STATUS  = 0x61;
        static const uint8_t RA_I2C_SLV0_DO        = 0x63;
        static const uint8_t RA_I2C_SLV1_DO        = 0x64;
        static const uint8_t RA_I2C_SLV2_DO        = 0x65;
        static const uint8_t RA_I2C_SLV3_DO        = 0x66;
        static const uint8_t RA_I2C_MST_DELAY_CTRL = 0x67;
        static const uint8_t RA_SIGNAL_PATH_RESET  = 0x68;
        static const uint8_t RA_MOT_DETECT_CTRL    = 0x69;
        static const uint8_t RA_USER_CTRL          = 0x6A;
        static const uint8_t RA_PWR_MGMT_1         = 0x6B;
        static const uint8_t RA_PWR_MGMT_2         = 0x6C;
        static const uint8_t RA_BANK_SEL           = 0x6D;
        static const uint8_t RA_MEM_START_ADDR     = 0x6E;
        static const uint8_t RA_MEM_R_W            = 0x6F;
        static const uint8_t RA_DMP_CFG_1          = 0x70;
        static const uint8_t RA_DMP_CFG_2          = 0x71;
        static const uint8_t RA_FIFO_COUNTH        = 0x72;
        static const uint8_t RA_FIFO_COUNTL        = 0x73;
        static const uint8_t RA_FIFO_R_W           = 0x74;

        // The gyro buffer is split 50/50, the first half for the transmit buffer, the
        // second half for the receive buffer This buffer is large enough for the gyros
        // currently supported in imu_mpu.c but should be reviewed id other gyro
        // types are supported with SPI DMA.
        static const uint8_t GYRO_BUF_SIZE = 32;


        enum gyro_fsr_e {
            INV_FSR_250DPS = 0,
            INV_FSR_500DPS,
            INV_FSR_1000DPS,
            INV_FSR_2000DPS,
            NUM_GYRO_FSR
        };

        enum icm_high_range_gyro_fsr_e {
            ICM_HIGH_RANGE_FSR_500DPS = 0,
            ICM_HIGH_RANGE_FSR_1000DPS,
            ICM_HIGH_RANGE_FSR_2000DPS,
            ICM_HIGH_RANGE_FSR_4000DPS,
            NUM_ICM_HIGH_RANGE_GYRO_FSR
        };

        enum clock_sel_e {
            INV_CLK_INTERNAL = 0,
            INV_CLK_PLL,
            NUM_CLK
        };

        enum accel_fsr_e {
            INV_FSR_2G = 0,
            INV_FSR_4G,
            INV_FSR_8G,
            INV_FSR_16G,
            NUM_ACCEL_FSR
        };

        enum icm_high_range_accel_fsr_e {
            ICM_HIGH_RANGE_FSR_4G = 0,
            ICM_HIGH_RANGE_FSR_8G,
            ICM_HIGH_RANGE_FSR_16G,
            ICM_HIGH_RANGE_FSR_32G,
            NUM_ICM_HIGH_RANGE_ACCEL_FSR
        };

        typedef enum {
            GYRO_OVERFLOW_NONE = 0x00,
            GYRO_OVERFLOW_X = 0x01,
            GYRO_OVERFLOW_Y = 0x02,
            GYRO_OVERFLOW_Z = 0x04
        } gyroOverflow_e;

        typedef struct {
            int8_t index;
            busType_e busType;
            uint8_t spiBus;
            ioTag_t csnTag;
            uint8_t i2cBus;
            uint8_t i2cAddress;
            ioTag_t extiTag;
            uint8_t alignment;
        } gyroDeviceConfig_t;

        static void gyroInit(void);

        static bool gyroRead(void);

        static bool gyroReadSPI(void);

        MpuImu(uint8_t interruptPin, uint16_t gyroScale)
            : FusionImu(interruptPin, gyroScale)
        {
        }

        virtual void devInit(uint8_t interruptPin) override;

        virtual bool devGyroIsReady(void) override;

}; // class MpuImu

mpuSensor_e mpuBusDetect(const extDevice_t *dev);
bool        mpuBusGyroDetect(gyroDev_t *gyro);
