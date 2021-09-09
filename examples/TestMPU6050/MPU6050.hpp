#include <stdint.h>

class MPU6050
{
    public:

        typedef enum {

            AFS_2G = 0,
            AFS_4G,
            AFS_8G,
            AFS_16G

        } ascale_t;

        typedef enum {

            GFS_250DPS = 0,
            GFS_500DPS,
            GFS_1000DPS,
            GFS_2000DPS

        } gscale_t;

        MPU6050(ascale_t ascale, gscale_t gscale);

        float getGres();

        float getAres();

        void readAccelData(int16_t * destination);

        void readGyroData(int16_t * destination);

        int16_t readTempData();

        void LowPowerAccelOnlyMPU6050();

        void begin();

        void calibrateMPU6050(float * dest1, float * dest2);

        void MPU6050SelfTest(float * destination);

        bool dataReady(void);

    private:

        static const uint8_t  MPU6050_ADDRESS     = 0x68;  // Device address when ADO   = 1;

        static const uint8_t  XGOFFS_TC           = 0x00; // Bit 7 PWR_MODE, bits 6:1 XG_OFFS_TC, bit 0 OTP_BNK_VLD;
        static const uint8_t  YGOFFS_TC           = 0x01;
        static const uint8_t  ZGOFFS_TC           = 0x02;
        static const uint8_t  X_FINE_GAIN         = 0x03; // [7:0] fine gain;
        static const uint8_t  Y_FINE_GAIN         = 0x04;
        static const uint8_t  Z_FINE_GAIN         = 0x05;
        static const uint8_t  XA_OFFSET_H         = 0x06; // User-defined trim values for accelerometer;
        static const uint8_t  XA_OFFSET_L_TC      = 0x07;
        static const uint8_t  YA_OFFSET_H         = 0x08;
        static const uint8_t  YA_OFFSET_L_TC      = 0x09;
        static const uint8_t  ZA_OFFSET_H         = 0x0A;
        static const uint8_t  ZA_OFFSET_L_TC      = 0x0B;
        static const uint8_t  SELF_TEST_X         = 0x0D;
        static const uint8_t  SELF_TEST_Y         = 0x0E;
        static const uint8_t  SELF_TEST_Z         = 0x0F;
        static const uint8_t  SELF_TEST_A         = 0x10;
        static const uint8_t  XG_OFFS_USRH        = 0x13;  // User-defined trim values for gyroscope; supported in MPU-6050?;
        static const uint8_t  XG_OFFS_USRL        = 0x14;
        static const uint8_t  YG_OFFS_USRH        = 0x15;
        static const uint8_t  YG_OFFS_USRL        = 0x16;
        static const uint8_t  ZG_OFFS_USRH        = 0x17;
        static const uint8_t  ZG_OFFS_USRL        = 0x18;
        static const uint8_t  SMPLRT_DIV          = 0x19;
        static const uint8_t  CONFIG              = 0x1A;
        static const uint8_t  GYRO_CONFIG         = 0x1B;
        static const uint8_t  ACCEL_CONFIG        = 0x1C;
        static const uint8_t  FF_THR              = 0x1D;  // Free-fall;
        static const uint8_t  FF_DUR              = 0x1E;  // Free-fall;
        static const uint8_t  MOT_THR             = 0x1F;  // Motion detection threshold bits [7:0];
        static const uint8_t  MOT_DUR             = 0x20;  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB   = 1 ms;
        static const uint8_t  ZMOT_THR            = 0x21;  // Zero-motion detection threshold bits [7:0];
        static const uint8_t  ZRMOT_DUR           = 0x22;  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB   = 64 ms;
        static const uint8_t  FIFO_EN             = 0x23;
        static const uint8_t  I2C_MST_CTRL        = 0x24;
        static const uint8_t  I2C_SLV0_ADDR       = 0x25;
        static const uint8_t  I2C_SLV0_REG        = 0x26;
        static const uint8_t  I2C_SLV0_CTRL       = 0x27;
        static const uint8_t  I2C_SLV1_ADDR       = 0x28;
        static const uint8_t  I2C_SLV1_REG        = 0x29;
        static const uint8_t  I2C_SLV1_CTRL       = 0x2A;
        static const uint8_t  I2C_SLV2_ADDR       = 0x2B;
        static const uint8_t  I2C_SLV2_REG        = 0x2C;
        static const uint8_t  I2C_SLV2_CTRL       = 0x2D;
        static const uint8_t  I2C_SLV3_ADDR       = 0x2E;
        static const uint8_t  I2C_SLV3_REG        = 0x2F;
        static const uint8_t  I2C_SLV3_CTRL       = 0x30;
        static const uint8_t  I2C_SLV4_ADDR       = 0x31;
        static const uint8_t  I2C_SLV4_REG        = 0x32;
        static const uint8_t  I2C_SLV4_DO         = 0x33;
        static const uint8_t  I2C_SLV4_CTRL       = 0x34;
        static const uint8_t  I2C_SLV4_DI         = 0x35;
        static const uint8_t  I2C_MST_STATUS      = 0x36;
        static const uint8_t  INT_PIN_CFG         = 0x37;
        static const uint8_t  INT_ENABLE          = 0x38;
        static const uint8_t  DMP_INT_STATUS      = 0x39;  // Check DMP interrupt;
        static const uint8_t  INT_STATUS          = 0x3A;
        static const uint8_t  ACCEL_XOUT_H        = 0x3B;
        static const uint8_t  ACCEL_XOUT_L        = 0x3C;
        static const uint8_t  ACCEL_YOUT_H        = 0x3D;
        static const uint8_t  ACCEL_YOUT_L        = 0x3E;
        static const uint8_t  ACCEL_ZOUT_H        = 0x3F;
        static const uint8_t  ACCEL_ZOUT_L        = 0x40;
        static const uint8_t  TEMP_OUT_H          = 0x41;
        static const uint8_t  TEMP_OUT_L          = 0x42;
        static const uint8_t  GYRO_XOUT_H         = 0x43;
        static const uint8_t  GYRO_XOUT_L         = 0x44;
        static const uint8_t  GYRO_YOUT_H         = 0x45;
        static const uint8_t  GYRO_YOUT_L         = 0x46;
        static const uint8_t  GYRO_ZOUT_H         = 0x47;
        static const uint8_t  GYRO_ZOUT_L         = 0x48;
        static const uint8_t  EXT_SENS_DATA_00    = 0x49;
        static const uint8_t  EXT_SENS_DATA_01    = 0x4A;
        static const uint8_t  EXT_SENS_DATA_02    = 0x4B;
        static const uint8_t  EXT_SENS_DATA_03    = 0x4C;
        static const uint8_t  EXT_SENS_DATA_04    = 0x4D;
        static const uint8_t  EXT_SENS_DATA_05    = 0x4E;
        static const uint8_t  EXT_SENS_DATA_06    = 0x4F;
        static const uint8_t  EXT_SENS_DATA_07    = 0x50;
        static const uint8_t  EXT_SENS_DATA_08    = 0x51;
        static const uint8_t  EXT_SENS_DATA_09    = 0x52;
        static const uint8_t  EXT_SENS_DATA_10    = 0x53;
        static const uint8_t  EXT_SENS_DATA_11    = 0x54;
        static const uint8_t  EXT_SENS_DATA_12    = 0x55;
        static const uint8_t  EXT_SENS_DATA_13    = 0x56;
        static const uint8_t  EXT_SENS_DATA_14    = 0x57;
        static const uint8_t  EXT_SENS_DATA_15    = 0x58;
        static const uint8_t  EXT_SENS_DATA_16    = 0x59;
        static const uint8_t  EXT_SENS_DATA_17    = 0x5A;
        static const uint8_t  EXT_SENS_DATA_18    = 0x5B;
        static const uint8_t  EXT_SENS_DATA_19    = 0x5C;
        static const uint8_t  EXT_SENS_DATA_20    = 0x5D;
        static const uint8_t  EXT_SENS_DATA_21    = 0x5E;
        static const uint8_t  EXT_SENS_DATA_22    = 0x5F;
        static const uint8_t  EXT_SENS_DATA_23    = 0x60;
        static const uint8_t  MOT_DETECT_STATUS   = 0x61;
        static const uint8_t  I2C_SLV0_DO         = 0x63;
        static const uint8_t  I2C_SLV1_DO         = 0x64;
        static const uint8_t  I2C_SLV2_DO         = 0x65;
        static const uint8_t  I2C_SLV3_DO         = 0x66;
        static const uint8_t  I2C_MST_DELAY_CTRL  = 0x67;
        static const uint8_t  SIGNAL_PATH_RESET   = 0x68;
        static const uint8_t  MOT_DETECT_CTRL     = 0x69;
        static const uint8_t  USER_CTRL           = 0x6A;  // Bit 7 enable DMP, bit 3 reset DMP;
        static const uint8_t  PWR_MGMT_1          = 0x6B; // Device defaults to the SLEEP mode;
        static const uint8_t  PWR_MGMT_2          = 0x6C;
        static const uint8_t  DMP_BANK            = 0x6D;  // Activates a specific bank in the DMP;
        static const uint8_t  DMP_RW_PNT          = 0x6E;  // Set read/write pointer to a specific start address in specified DMP bank;
        static const uint8_t  DMP_REG             = 0x6F;  // Register in DMP from which to read or to which to write;
        static const uint8_t  DMP_REG_1           = 0x70;
        static const uint8_t  DMP_REG_2           = 0x71;
        static const uint8_t  FIFO_COUNTH         = 0x72;
        static const uint8_t  FIFO_COUNTL         = 0x73;
        static const uint8_t  FIFO_R_W            = 0x74;
        static const uint8_t  WHO_AM_I_MPU6050    = 0x75; // Should return  = 0x68;

        ascale_t _ascale;
        gscale_t _gscale;

        void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);

        uint8_t readByte(uint8_t address, uint8_t subAddress);

        void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
};
