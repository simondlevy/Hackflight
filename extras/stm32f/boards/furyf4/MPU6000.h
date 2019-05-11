/*
   MPU6000.h : Experimental class for Invensense MPU6000 IMU using SPI bus

   Copyright (C) 2019 Simon D. Levy 

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>

class MPU6000 {

    public:

        typedef enum {

            INV_CLK_INTERNAL,
            INV_CLK_PLL

        } Clock_e;

        typedef enum {

            AFS_2G,
            AFS_4G,  
            AFS_8G,  
            AFS_16G 

        } Ascale_t;

        typedef enum {

            GFS_250DPS,
            GFS_500DPS,
            GFS_1000DPS,
            GFS_2000DPS

        } Gscale_t;

        typedef enum {

            ERROR_NONE,
            ERROR_CONNECT,
            ERROR_IMU_ID,
            ERROR_MAG_ID,
            ERROR_SELFTEST

        } Error_t;



    private:

        const uint8_t MPU_ADDRESS               = 0x68;

        // Register map
        static const uint8_t SELF_TEST_X_ACCEL  = 0x0D;
        static const uint8_t SELF_TEST_Y_ACCEL  = 0x0E;    
        static const uint8_t SELF_TEST_Z_ACCEL  = 0x0F;
        static const uint8_t SELF_TEST_A        = 0x10;
        static const uint8_t XG_OFFSET_H        = 0x13; 
        static const uint8_t XG_OFFSET_L        = 0x14;
        static const uint8_t YG_OFFSET_H        = 0x15;
        static const uint8_t YG_OFFSET_L        = 0x16;
        static const uint8_t ZG_OFFSET_H        = 0x17;
        static const uint8_t ZG_OFFSET_L        = 0x18;
        static const uint8_t SMPLRT_DIV         = 0x19;
        static const uint8_t I2C_MST_EN         = 0x20;
        static const uint8_t ACCEL_CONFIG2      = 0x1D;
        static const uint8_t LP_ACCEL_ODR       = 0x1E;
        static const uint8_t MOT_THR            = 0x1F;   
        static const uint8_t MOT_DUR            = 0x20;  
        static const uint8_t CONFIG             = 0x1A;
        static const uint8_t GYRO_CONFIG        = 0x1B;
        static const uint8_t ACCEL_CONFIG       = 0x1C;
        static const uint8_t ZMOT_THR           = 0x21;  
        static const uint8_t ZRMOT_DUR          = 0x22;  
        static const uint8_t FIFO_EN            = 0x23;
        static const uint8_t I2C_MST_CTRL       = 0x24;
        static const uint8_t I2C_SLV0_ADDR      = 0x25;
        static const uint8_t I2C_SLV0_REG       = 0x26;
        static const uint8_t I2C_SLV0_CTRL      = 0x27;
        static const uint8_t I2C_SLV1_ADDR      = 0x28;
        static const uint8_t I2C_SLV1_REG       = 0x29;
        static const uint8_t I2C_SLV1_CTRL      = 0x2A;
        static const uint8_t I2C_SLV2_ADDR      = 0x2B;
        static const uint8_t I2C_SLV2_REG       = 0x2C;
        static const uint8_t I2C_SLV2_CTRL      = 0x2D;
        static const uint8_t I2C_SLV3_ADDR      = 0x2E;
        static const uint8_t I2C_SLV3_REG       = 0x2F;
        static const uint8_t I2C_SLV3_CTRL      = 0x30;
        static const uint8_t I2C_SLV4_ADDR      = 0x31;
        static const uint8_t I2C_SLV4_REG       = 0x32;
        static const uint8_t I2C_SLV4_DO        = 0x33;
        static const uint8_t I2C_SLV4_CTRL      = 0x34;
        static const uint8_t I2C_SLV4_DI        = 0x35;
        static const uint8_t I2C_MST_STATUS     = 0x36;
        static const uint8_t INT_PIN_CFG        = 0x37;
        static const uint8_t INT_ENABLE         = 0x38;
        static const uint8_t DMP_INT_STATUS     = 0x39;  // Check DMP interrupt
        static const uint8_t INT_STATUS         = 0x3A;
        static const uint8_t ACCEL_XOUT_H       = 0x3B;
        static const uint8_t ACCEL_XOUT_L       = 0x3C;
        static const uint8_t ACCEL_YOUT_H       = 0x3D;
        static const uint8_t ACCEL_YOUT_L       = 0x3E;
        static const uint8_t ACCEL_ZOUT_H       = 0x3F;
        static const uint8_t ACCEL_ZOUT_L       = 0x40;
        static const uint8_t TEMP_OUT_H         = 0x41;
        static const uint8_t TEMP_OUT_L         = 0x42;
        static const uint8_t GYRO_XOUT_H        = 0x43;
        static const uint8_t GYRO_XOUT_L        = 0x44;
        static const uint8_t GYRO_YOUT_H        = 0x45;
        static const uint8_t GYRO_YOUT_L        = 0x46;
        static const uint8_t GYRO_ZOUT_H        = 0x47;
        static const uint8_t GYRO_ZOUT_L        = 0x48;
        static const uint8_t EXT_SENS_DATA_00   = 0x49;
        static const uint8_t EXT_SENS_DATA_01   = 0x4A;
        static const uint8_t EXT_SENS_DATA_02   = 0x4B;
        static const uint8_t EXT_SENS_DATA_03   = 0x4C;
        static const uint8_t EXT_SENS_DATA_04   = 0x4D;
        static const uint8_t EXT_SENS_DATA_05   = 0x4E;
        static const uint8_t EXT_SENS_DATA_06   = 0x4F;
        static const uint8_t EXT_SENS_DATA_07   = 0x50;
        static const uint8_t EXT_SENS_DATA_08   = 0x51;
        static const uint8_t EXT_SENS_DATA_09   = 0x52;
        static const uint8_t EXT_SENS_DATA_10   = 0x53;
        static const uint8_t EXT_SENS_DATA_11   = 0x54;
        static const uint8_t EXT_SENS_DATA_12   = 0x55;
        static const uint8_t EXT_SENS_DATA_13   = 0x56;
        static const uint8_t EXT_SENS_DATA_14   = 0x57;
        static const uint8_t EXT_SENS_DATA_15   = 0x58;
        static const uint8_t EXT_SENS_DATA_16   = 0x59;
        static const uint8_t EXT_SENS_DATA_17   = 0x5A;
        static const uint8_t EXT_SENS_DATA_18   = 0x5B;
        static const uint8_t EXT_SENS_DATA_19   = 0x5C;
        static const uint8_t EXT_SENS_DATA_20   = 0x5D;
        static const uint8_t EXT_SENS_DATA_21   = 0x5E;
        static const uint8_t EXT_SENS_DATA_22   = 0x5F;
        static const uint8_t EXT_SENS_DATA_23   = 0x60;
        static const uint8_t MOT_DETECT_STATUS  = 0x61;
        static const uint8_t I2C_SLV0_DO        = 0x63;
        static const uint8_t I2C_SLV1_DO        = 0x64;
        static const uint8_t I2C_SLV2_DO        = 0x65;
        static const uint8_t I2C_SLV3_DO        = 0x66;
        static const uint8_t I2C_MST_DELAY_CTRL = 0x67;
        static const uint8_t SIGNAL_PATH_RESET  = 0x68;
        static const uint8_t MOT_DETECT_CTRL    = 0x69;
        static const uint8_t USER_CTRL          = 0x6A;  
        static const uint8_t PWR_MGMT_1         = 0x6B; 
        static const uint8_t PWR_MGMT_2         = 0x6C;
        static const uint8_t DMP_BANK           = 0x6D; 
        static const uint8_t DMP_RW_PNT         = 0x6E;
        static const uint8_t DMP_REG            = 0x6F;
        static const uint8_t DMP_REG_1          = 0x70;
        static const uint8_t DMP_REG_2          = 0x71;
        static const uint8_t FIFO_COUNTH        = 0x72;
        static const uint8_t FIFO_COUNTL        = 0x73;
        static const uint8_t FIFO_R_W           = 0x74;
        static const uint8_t WHO_AM_I           = 0x75; 
        static const uint8_t XA_OFFSET_H        = 0x77;
        static const uint8_t XA_OFFSET_L        = 0x78;
        static const uint8_t YA_OFFSET_H        = 0x7A;
        static const uint8_t YA_OFFSET_L        = 0x7B;
        static const uint8_t ZA_OFFSET_H        = 0x7D;
        static const uint8_t ZA_OFFSET_L        = 0x7E;
        static const uint8_t I2C_SLV0_EN        = 0x80;
        static const uint8_t I2C_READ_FLAG      = 0x80;

        Ascale_t _aScale;
        Gscale_t _gScale;
        uint8_t  _sampleRateDivisor;

        float   _aRes;
        float   _gRes;
        float _accelBias[3];
        float _gyroBias[3];
 
        uint8_t readRegister(uint8_t subAddress);

        void readRegisters(uint8_t subAddress, uint8_t count, uint8_t * dest);

        void writeRegister(uint8_t subAddress, uint8_t data);

    public:

        MPU6000(Ascale_t ascale, Gscale_t gscale, uint8_t sampleRateDivisor=0);

        MPU6000::Error_t begin(void);

        uint8_t getId();

	bool readAccel(int16_t & x, int16_t & y, int16_t & z);

        bool readGyro(int16_t & x, int16_t & y, int16_t & z);


}; // class MPU6000 SPI
