/*
   Header-only class definition for MPU6000 sensor using SPI bus

   Copyright (c) 2022 Simon D. Levy

   MIT License
*/

#include "imus/real/softquat/spi.h"

#include <SPI.h>

class Mpu6000 : public SpiImu {

    public:

        typedef enum {

            GYRO_250DPS,
            GYRO_500DPS,
            GYRO_1000DPS,
            GYRO_2000DPS

        } gyroScale_e;

        typedef enum {

            ACCEL_2G,
            ACCEL_4G,  
            ACCEL_8G,  
            ACCEL_16G

        } accelScale_e;

    private:

        // Registers
        static const uint8_t REG_SMPLRT_DIV   = 0x19;
        static const uint8_t REG_CONFIG       = 0x1A;
        static const uint8_t REG_GYRO_CONFIG  = 0x1B;
        static const uint8_t REG_ACCEL_CONFIG = 0x1C;
        static const uint8_t REG_INT_PIN_CFG  = 0x37;
        static const uint8_t REG_INT_ENABLE   = 0x38;
        static const uint8_t REG_GYRO_XOUT_H  = 0x43;
        static const uint8_t REG_USER_CTRL    = 0x6A;
        static const uint8_t REG_PWR_MGMT_1   = 0x6B;
        static const uint8_t REG_PWR_MGMT_2   = 0x6C;
        static const uint8_t REG_WHO_AM_I     = 0x75;

        // Configuration bits  
        static const uint8_t BIT_RAW_RDY_EN       = 0x01;
        static const uint8_t BIT_CLK_SEL_PLLGYROZ = 0x03;
        static const uint8_t BIT_INT_ANYRD_2CLEAR = 0x10;
        static const uint8_t BIT_I2C_IF_DIS       = 0x10;
        static const uint8_t BIT_H_RESET          = 0x80;

        // Any interrupt interval less than this will be recognised as the
        // short interval of ~79us
        static const uint8_t SHORT_THRESHOLD = 82 ;

        // 1 MHz max SPI frequency for initialisation
        static const uint32_t MAX_SPI_INIT_CLK_HZ = 1000000;

        // 20 MHz max SPI frequency
        static const uint32_t MAX_SPI_CLK_HZ = 20000000;

        // Sample rate = 200Hz    Fsample= 1Khz/(4+1) = 200Hz     
        // Sample rate = 50Hz    Fsample= 1Khz/(19+1) = 50Hz     
        uint8_t m_sampleRateDivisor;

        gyroScale_e m_gyroScale;

        accelScale_e m_accelScale;

        int32_t m_shortPeriod;

        uint8_t m_buffer[15];

        int16_t getValue(const uint8_t k)
        {
            return (int16_t)(m_buffer[k] << 8 | m_buffer[k+1]);
        }

        uint16_t calculateSpiDivisor(const uint32_t freq)
        {
            uint32_t clk = m_board->getClockSpeed() / 2;

            uint16_t divisor = 2;

            clk >>= 1;

            for (; (clk > freq) && (divisor < 256); divisor <<= 1, clk >>= 1);

            return divisor;
        }

    protected:

        virtual bool gyroIsReady(void) override
        {
            // If we call this infrequently enough, gyro will always be ready
            readGyro();
            return true;
        }

        virtual void begin(void) override
        {
            m_shortPeriod = m_board->getClockSpeed() / 1000000 * SHORT_THRESHOLD;

            m_spi->begin();
            m_spi->setBitOrder(MSBFIRST);
            m_spi->setClockDivider(calculateSpiDivisor(MAX_SPI_INIT_CLK_HZ));
            m_spi->setDataMode(SPI_MODE3);

            pinMode(m_csPin, OUTPUT);

            // Chip reset
            writeRegister(REG_PWR_MGMT_1, BIT_H_RESET);
            delay(100);

            // Check ID
            readRegister(REG_WHO_AM_I);

            // Clock Source PPL with Z axis gyro reference
            writeRegister(REG_PWR_MGMT_1, BIT_CLK_SEL_PLLGYROZ);
            delayMicroseconds(7);

            // Disable Primary I2C Interface
            writeRegister(REG_USER_CTRL, BIT_I2C_IF_DIS);
            delayMicroseconds(15);

            writeRegister(REG_PWR_MGMT_2, 0x00);
            delayMicroseconds(15);

            // Accel Sample Rate 1kHz
            // Gyroscope Output Rate =  1kHz when the DLPF is enabled
            writeRegister(REG_SMPLRT_DIV, 0);
            delayMicroseconds(15);

            // Gyro +/- 2000 DPS Full Scale
            writeRegister(REG_GYRO_CONFIG, m_gyroScale << 3);
            delayMicroseconds(15);

            // Accel +/- 16 G Full Scale
            writeRegister(REG_ACCEL_CONFIG, m_accelScale << 3);
            delayMicroseconds(15);

            // INT_ANYRD_2CLEAR
            writeRegister(REG_INT_PIN_CFG, 0x10);

            delayMicroseconds(15);

            writeRegister(REG_INT_ENABLE, BIT_RAW_RDY_EN);
            delayMicroseconds(15);

            m_spi->setClockDivider(calculateSpiDivisor(MAX_SPI_CLK_HZ));
            delayMicroseconds(1);

            m_spi->setClockDivider(calculateSpiDivisor(MAX_SPI_INIT_CLK_HZ));

            // Accel and Gyro DLPF Setting
            writeRegister(REG_CONFIG, 0); // no gyro DLPF
            delayMicroseconds(1);

            m_spi->setClockDivider(calculateSpiDivisor(MAX_SPI_CLK_HZ));
        }

        virtual int16_t readRawGyro(uint8_t k) override
        {
            return getValue(1 + k*2);
        }

    public:

        Mpu6000(
                const rotateFun_t rotateFun,
                SPIClass & spi,
                const uint8_t csPin,
                const uint8_t sampleRateDivisor = 19,
                const gyroScale_e gyroScale = GYRO_2000DPS,
                const accelScale_e accelScale = ACCEL_2G)
            : SpiImu(
                    rotateFun, 
                    spi,
                    csPin,
                    (gyroScale == GYRO_250DPS ?  250 : 
                     gyroScale == GYRO_500DPS ?  500 : 
                     gyroScale == GYRO_1000DPS ?  1000 : 
                     2000) 
                    / 32768.)
            {
                m_sampleRateDivisor = sampleRateDivisor;
                m_gyroScale = gyroScale;
                m_accelScale = accelScale;
            }

        void handleInterrupt(void)
        {
            static uint32_t prevTime;

            // Ideally we'd use a time to capture such information, but
            // unfortunately the port used for EXTI interrupt does not have an
            // associated timer
            uint32_t nowCycles = m_board->getCycleCounter();
            int32_t gyroLastPeriod = cmpTimeCycles(nowCycles, prevTime);

            // This detects the short (~79us) EXTI interval of an MPU6xxx gyro
            if ((m_shortPeriod == 0) || (gyroLastPeriod < m_shortPeriod)) {

                m_gyroSyncTime = prevTime;
            }

            prevTime = nowCycles;

            RealImu::handleInterrupt();
        }

        void readGyro(void)
        {
            readRegisters(REG_GYRO_XOUT_H, m_buffer, 7);
        }

}; // class Mpu6000
