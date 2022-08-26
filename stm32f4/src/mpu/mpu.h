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
   Hackflight. If not, see <https:
 */

#include "imus/fusion.h"

#include "time.h"
#include "bus.h"
#include "bus_spi.h"
#include "devices.h"
#include "nvic.h"

class MpuImu : public FusionImu {

    private:

        static const uint8_t RA_WHO_AM_I_LEGACY         = 0x00;
        static const uint8_t RA_XG_OFFS_TC              = 0x00;
        static const uint8_t RA_YG_OFFS_TC              = 0x01;   
        static const uint8_t RA_ZG_OFFS_TC              = 0x02;    
        static const uint8_t RA_X_FINE_GAIN             = 0x03;  
        static const uint8_t RA_Y_FINE_GAIN             = 0x04;  
        static const uint8_t RA_Z_FINE_GAIN             = 0x05;  
        static const uint8_t RA_XA_OFFS_H               = 0x06;  
        static const uint8_t RA_XA_OFFS_L_TC     		= 0x07;
        static const uint8_t RA_YA_OFFS_H        		= 0x08;  
        static const uint8_t RA_YA_OFFS_L_TC     		= 0x09;
        static const uint8_t RA_ZA_OFFS_H        		= 0x0A;   
        static const uint8_t RA_ZA_OFFS_L_TC     		= 0x0B;
        static const uint8_t RA_XG_OFFS_USRH     		= 0x13;    
        static const uint8_t RA_XG_OFFS_USRL     		= 0x14;
        static const uint8_t RA_YG_OFFS_USRH     		= 0x15;    
        static const uint8_t RA_YG_OFFS_USRL     		= 0x16;
        static const uint8_t RA_ZG_OFFS_USRH     		= 0x17;    
        static const uint8_t RA_ZG_OFFS_USRL     		= 0x18;
        static const uint8_t RA_SMPLRT_DIV       		= 0x19;
        static const uint8_t RA_CONFIG           		= 0x1A;
        static const uint8_t RA_GYRO_CONFIG      		= 0x1B;
        static const uint8_t RA_ACCEL_CONFIG     		= 0x1C;
        static const uint8_t RA_FF_THR           		= 0x1D;
        static const uint8_t RA_FF_DUR           		= 0x1E;
        static const uint8_t RA_MOT_THR          		= 0x1F;
        static const uint8_t RA_MOT_DUR          		= 0x20;
        static const uint8_t RA_ZRMOT_THR        		= 0x21;
        static const uint8_t RA_ZRMOT_DUR        		= 0x22;
        static const uint8_t RA_FIFO_EN          		= 0x23;
        static const uint8_t RA_I2C_MST_CTRL     		= 0x24;
        static const uint8_t RA_I2C_SLV0_ADDR    		= 0x25;
        static const uint8_t RA_I2C_SLV0_REG     		= 0x26;
        static const uint8_t RA_I2C_SLV0_CTRL    		= 0x27;
        static const uint8_t RA_I2C_SLV1_ADDR    		= 0x28;
        static const uint8_t RA_I2C_SLV1_REG     		= 0x29;
        static const uint8_t RA_I2C_SLV1_CTRL    		= 0x2A;
        static const uint8_t RA_I2C_SLV2_ADDR    		= 0x2B;
        static const uint8_t RA_I2C_SLV2_REG     		= 0x2C;
        static const uint8_t RA_I2C_SLV2_CTRL    		= 0x2D;
        static const uint8_t RA_I2C_SLV3_ADDR    		= 0x2E;
        static const uint8_t RA_I2C_SLV3_REG     		= 0x2F;
        static const uint8_t RA_I2C_SLV3_CTRL    		= 0x30;
        static const uint8_t RA_I2C_SLV4_ADDR    		= 0x31;
        static const uint8_t RA_I2C_SLV4_REG     		= 0x32;
        static const uint8_t RA_I2C_SLV4_DO      		= 0x33;
        static const uint8_t RA_I2C_SLV4_CTRL    		= 0x34;
        static const uint8_t RA_I2C_SLV4_DI      		= 0x35;
        static const uint8_t RA_I2C_MST_STATUS   		= 0x36;
        static const uint8_t RA_INT_PIN_CFG      		= 0x37;
        static const uint8_t RA_INT_ENABLE       		= 0x38;
        static const uint8_t RA_DMP_INT_STATUS   		= 0x39;
        static const uint8_t RA_INT_STATUS       		= 0x3A;
        static const uint8_t RA_ACCEL_XOUT_H     		= 0x3B;
        static const uint8_t RA_ACCEL_XOUT_L     		= 0x3C;
        static const uint8_t RA_ACCEL_YOUT_H     		= 0x3D;
        static const uint8_t RA_ACCEL_YOUT_L     		= 0x3E;
        static const uint8_t RA_ACCEL_ZOUT_H     		= 0x3F;
        static const uint8_t RA_ACCEL_ZOUT_L     		= 0x40;
        static const uint8_t RA_TEMP_OUT_H       		= 0x41;
        static const uint8_t RA_TEMP_OUT_L       		= 0x42;
        static const uint8_t RA_GYRO_XOUT_H      		= 0x43;
        static const uint8_t RA_GYRO_XOUT_L      		= 0x44;
        static const uint8_t RA_GYRO_YOUT_H      		= 0x45;
        static const uint8_t RA_GYRO_YOUT_L      		= 0x46;
        static const uint8_t RA_GYRO_ZOUT_H      		= 0x47;
        static const uint8_t RA_GYRO_ZOUT_L      		= 0x48;
        static const uint8_t RA_EXT_SENS_DATA_00 		= 0x49;
        static const uint8_t RA_MOT_DETECT_STATUS       = 0x61;
        static const uint8_t RA_I2C_SLV0_DO             = 0x63;
        static const uint8_t RA_I2C_SLV1_DO             = 0x64;
        static const uint8_t RA_I2C_SLV2_DO             = 0x65;
        static const uint8_t RA_I2C_SLV3_DO             = 0x66;
        static const uint8_t RA_I2C_MST_DELAY_CTRL      = 0x67;
        static const uint8_t RA_MOT_DETECT_CTRL         = 0x69;
        static const uint8_t RA_USER_CTRL               = 0x6A;
        static const uint8_t RA_PWR_MGMT_2              = 0x6C;
        static const uint8_t RA_BANK_SEL                = 0x6D;
        static const uint8_t RA_MEM_START_ADDR          = 0x6E;
        static const uint8_t RA_MEM_R_W                 = 0x6F;
        static const uint8_t RA_DMP_CFG_1               = 0x70;
        static const uint8_t RA_DMP_CFG_2               = 0x71;
        static const uint8_t RA_FIFO_COUNTH             = 0x72;
        static const uint8_t RA_FIFO_COUNTL             = 0x73;
        static const uint8_t RA_FIFO_R_W                = 0x74;

        // Need to see at least this many interrupts during initialisation to
        // confirm EXTI connectivity
        static const uint16_t GYRO_EXTI_DETECT_THRESHOLD = 1000;

        // The gyro buffer is split 50/50, the first half for the transmit
        // buffer, the second half for the receive buffer This buffer is large
        // enough for the gyros currently supported in imu_mpu.c but should be
        // reviewed id other gyro types are supported with SPI DMA.
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

        typedef struct gyroDeviceConfig_s {
            int8_t index;
            busType_e busType;
            uint8_t spiBus;
            ioTag_t csnTag;
            uint8_t i2cBus;
            uint8_t i2cAddress;
            ioTag_t extiTag;
            uint8_t alignment;        
        } gyroDeviceConfig_t;

    protected:

        static const uint8_t WHO_AM_I_CONST       = 0x68;
        static const uint8_t RA_WHO_AM_I          = 0x75;
        static const uint8_t RA_PWR_MGMT_1        = 0x6B;
        static const uint8_t RA_SIGNAL_PATH_RESET = 0x68;
        static const uint8_t RA_PRODUCT_ID        = 0x0C;   

        MpuImu(uint8_t interruptPin) 
            : FusionImu(interruptPin)
        {
        }

        static void intExtiHandler(extiCallbackRec_t *cb)
        {
            gyroDev_t *gyroDev = gyroContainerOf(cb);

            // Ideally we'd use a time to capture such information, but
            // unfortunately the port used for EXTI interrupt does not have an
            // associated timer
            uint32_t nowCycles = systemGetCycleCounter();
            int32_t gyroLastPeriod = cmpTimeCycles(nowCycles, gyroDev->gyroLastEXTI);
            // This detects the short (~79us) EXTI interval of an MPU6xxx gyro
            if ((gyroDev->gyroShortPeriod == 0) ||
                    (gyroLastPeriod < gyroDev->gyroShortPeriod)) {
                gyroDev->gyroSyncEXTI =
                    gyroDev->gyroLastEXTI + gyroDev->gyroDmaMaxDuration;
            }
            gyroDev->gyroLastEXTI = nowCycles;

            gyroDev->detectedEXTI++;
        }

        bool accRead(accDev_t *acc)
        {
            uint8_t data[6];

            const bool ack =
                busReadRegisterBuffer(&acc->gyro->dev, RA_ACCEL_XOUT_H, data, 6);
            if (!ack) {
                return false;
            }

            acc->ADCRaw[0] = (int16_t)((data[0] << 8) | data[1]);
            acc->ADCRaw[1] = (int16_t)((data[2] << 8) | data[3]);
            acc->ADCRaw[2] = (int16_t)((data[4] << 8) | data[5]);

            return true;
        }

        bool gyroRead(gyroDev_t *gyro)
        {
            uint8_t data[6];

            const bool ack = busReadRegisterBuffer(&gyro->dev, RA_GYRO_XOUT_H, data, 6);
            if (!ack) {
                return false;
            }

            gyro->adcRaw[0] = (int16_t)((data[0] << 8) | data[1]);
            gyro->adcRaw[1] = (int16_t)((data[2] << 8) | data[3]);
            gyro->adcRaw[2] = (int16_t)((data[4] << 8) | data[5]);

            return true;
        }

        bool accReadSPI(accDev_t *acc)
        {
            // Ensure any prior DMA has completed before continuing
            spiWaitClaim(&acc->gyro->dev);

            acc->gyro->dev.txBuf[0] = RA_ACCEL_XOUT_H | 0x80;

            busSegment_t segments[] = {
                {NULL, NULL, 7, true, NULL},
                {NULL, NULL, 0, true, NULL},
            };
            segments[0].txData = acc->gyro->dev.txBuf;
            segments[0].rxData = &acc->gyro->dev.rxBuf[1];

            spiSequence(&acc->gyro->dev, &segments[0]);

            // Wait for completion
            spiWait(&acc->gyro->dev);

            return true;
        }

        bool gyroReadSPI(gyroDev_t *gyro)
        {
            uint16_t *gyroData = (uint16_t *)gyro->dev.rxBuf;

            // Ensure any prior DMA has completed before continuing
            spiWaitClaim(&gyro->dev);

            gyro->dev.txBuf[0] = RA_GYRO_XOUT_H | 0x80;

            busSegment_t segments[] = {
                {NULL, NULL, 7, true, NULL},
                {NULL, NULL, 0, true, NULL},
            };
            segments[0].txData = gyro->dev.txBuf;
            segments[0].rxData = &gyro->dev.rxBuf[1];

            spiSequence(&gyro->dev, &segments[0]);

            // Wait for completion
            spiWait(&gyro->dev);

            gyro->adcRaw[0] = __builtin_bswap16(gyroData[1]);
            gyro->adcRaw[1] = __builtin_bswap16(gyroData[2]);
            gyro->adcRaw[2] = __builtin_bswap16(gyroData[3]);

            return true;
        }

        typedef uint8_t (*gyroSpiDetectFn_t)(const extDevice_t *dev);

        bool detectSPISensorsAndUpdateDetectionResult(gyroDev_t *gyro,
                const gyroDeviceConfig_t *config)
        {
            if (!config->csnTag || !spiSetBusInstance(&gyro->dev, config->spiBus)) {
                return false;
            }

            gyro->dev.busType_u.spi.csnPin = IOGetByTag(config->csnTag);

            IOInit(gyro->dev.busType_u.spi.csnPin, OWNER_GYRO_CS,
                    RESOURCE_INDEX(config->index));
            IOConfigGPIO(gyro->dev.busType_u.spi.csnPin, SPI_IO_CS_CFG);

            // Ensure device is disabled, important when two devices are on the same bus.
            IOHi(gyro->dev.busType_u.spi.csnPin); 

            // It is hard to use hardware to optimize the detection loop here,
            // as hardware type and detection function name doesn't match.
            // May need a bitmap of hardware to detection function to do it right?
            mpuSensor_e sensor = busDetect(&gyro->dev);
            if (sensor != MPU_NONE) {
                gyro->mpuDetectionResult.sensor = sensor;
                busDeviceRegister(&gyro->dev);
                return true;
            }

            // Detection failed, disable CS pin again
            spiPreinitByTag(config->csnTag);

            return false;
        }

        void preInit(gyroDeviceConfig_t * config)
        {
            (void)config;
            spiPreinitRegister(config->csnTag, IOCFG_IPU, 1);
        }

        bool detect(gyroDev_t *gyro, const gyroDeviceConfig_t *config)
        {
            static busDevice_t bus;
            gyro->dev.bus = &bus;

            // MPU datasheet specifies 30ms.
            delayMillis(35);

            if (config->busType == BUS_TYPE_NONE) {
                return false;
            }

            if (config->busType == BUS_TYPE_GYRO_AUTO) {
                gyro->dev.bus->busType = BUS_TYPE_I2C;
            } else {
                gyro->dev.bus->busType = config->busType;
            }

            gyro->dev.bus->busType = BUS_TYPE_SPI;

            return detectSPISensorsAndUpdateDetectionResult(gyro, config);
        }

        void gyroInit(gyroDev_t *gyro)
        {
            if (gyro->mpuIntExtiTag == IO_TAG_NONE) {
                return;
            }

            const IO_t mpuIntIO = IOGetByTag(gyro->mpuIntExtiTag);

            IOInit(mpuIntIO, OWNER_GYRO_EXTI, 0);
            EXTIHandlerInit(&gyro->exti, intExtiHandler);
            EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, IOCFG_IN_FLOATING,
                    BETAFLIGHT_EXTI_TRIGGER_RISING);
            EXTIEnable(mpuIntIO, true);
        }

        uint8_t gyroDLPF(gyroDev_t *gyro)
        {
            (void)gyro;
            return 0;
        }

        uint8_t gyroReadRegister(const extDevice_t *dev, uint8_t reg)
        {
            uint8_t data;
            const bool ack = busReadRegisterBuffer(dev, reg, &data, 1);
            if (ack) {
                return data;
            } else {
                return 0;
            }

        }

        accDev_t  m_accelDev;
        gyroDev_t m_gyroDev;

        const mpuDetectionResult_t *gyroMpuDetectionResult(void)
        {
            return &m_gyroDev.mpuDetectionResult;
        }

        virtual mpuSensor_e busDetect(const extDevice_t *dev) = 0;

        virtual bool busAccDetect(accDev_t *acc) = 0;

        virtual bool busGyroDetect(gyroDev_t *gyro) = 0;

        virtual bool readAcc(accDev_t * acc) = 0;

        virtual bool readGyro(gyroDev_t * gyro) = 0;

    public:

        virtual uint32_t devGyroInterruptCount(void) override
        {
            return m_gyroDev.detectedEXTI;
        }

        virtual bool devGyroIsReady(void) override
        {
            bool ready = m_gyroDev.readFn(&m_gyroDev);

            if (ready) {
                m_gyroDev.dataReady = false;
            }

            return ready;
        }

        virtual int16_t devReadRawGyro(uint8_t k) override
        {
            return m_gyroDev.adcRaw[k];
        }

        virtual uint32_t devGyroSyncTime(void) override
        {
            return m_gyroDev.gyroSyncEXTI;
        }

        virtual void devInit(uint8_t interruptPin) override
        {
            (void)interruptPin;

            static gyroDeviceConfig_t gyroDeviceConfig; 

            gyroDeviceConfig.busType = BUS_TYPE_SPI; // XXX pass from subclass
            gyroDeviceConfig.spiBus = 1;
            gyroDeviceConfig.csnTag = 20;
            gyroDeviceConfig.extiTag = 52;

            spiPreinitRegister(gyroDeviceConfig.csnTag, IOCFG_IPU, 1);

            detect(&m_gyroDev, &gyroDeviceConfig);
            busGyroDetect(&m_gyroDev);

            // SPI DMA buffer required per device
            static uint8_t gyroBuf1[GYRO_BUF_SIZE];
            m_gyroDev.dev.txBuf = gyroBuf1;
            m_gyroDev.dev.rxBuf = &gyroBuf1[GYRO_BUF_SIZE / 2];

            m_gyroDev.mpuIntExtiTag = gyroDeviceConfig.extiTag;

            m_gyroDev.accSampleRateHz = 1000;// XXX pass from subclass
            m_gyroDev.gyroSampleRateHz = 8000;// XXX pass from subclass

            m_gyroDev.initFn(&m_gyroDev);

            m_accelDev.gyro = &m_gyroDev;
            m_accelDev.mpuDetectionResult = *gyroMpuDetectionResult();
            m_accelDev.acc_high_fsr = false;
            busAccDetect(&m_accelDev);
        }

        virtual uint16_t devScaleGyro(void) override
        {
            return m_gyroDev.scaleDps;
        }


};  // class MpuImu
