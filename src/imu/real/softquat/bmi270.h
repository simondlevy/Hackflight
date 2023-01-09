/*
   Class definition for MPU6000, MPU6500 IMUs using SPI bus

   Copyright (c) 2022 Simon D. Levy

   MIT License
 */

#include <SPI.h>

#include <stdint.h>

#include "imu/real/softquat.h"

class Bmi270 : public SoftQuatImu {

    private:

        // Registers
        static const uint8_t REG_CHIP_ID = 0x00;

        SPIClass m_spi;

        void writeRegister(const uint8_t reg, const uint8_t val)
        {
            digitalWrite(m_csPin, LOW);
            m_spi.transfer(reg);
            m_spi.transfer(val);
            digitalWrite(m_csPin, HIGH);
        }

        void readRegisters(
                const uint8_t addr, uint8_t * buffer, const uint8_t count)
        {
            digitalWrite(m_csPin, LOW);
            buffer[0] = addr | 0x80;
            m_spi.transfer(buffer, count+1);
            digitalWrite(m_csPin, HIGH);
        }

        uint8_t readRegister(const uint8_t addr)
        {
            uint8_t buffer[2] = {};
            readRegisters(addr, buffer, 1);
            return buffer[1];
        }

        void setClockDivider(uint32_t divider)
        {
            m_spi.setClockDivider(divider);
        }

    protected:

        uint8_t m_mosiPin;
        uint8_t m_misoPin;
        uint8_t m_sclkPin;
        uint8_t m_csPin;

        uint16_t calculateSpiDivisor(const uint32_t freq)
        {
            uint32_t clk = m_board->getClockSpeed() / 2;

            uint16_t divisor = 2;

            clk >>= 1;

            for (; (clk > freq) && (divisor < 256); divisor <<= 1, clk >>= 1);

            return divisor;
        }

        virtual bool gyroIsReady(void) override
        {

            readRegisters(REG_ACCEL_XOUT_H, m_buffer, 14);

            // If we call this infrequently enough, gyro will always be ready
            return true;
        }

        virtual void begin(void) override
        {
            m_spi.setMOSI(m_mosiPin);
            m_spi.setMISO(m_misoPin);
            m_spi.setSCLK(m_sclkPin);

            m_spi.begin();
            m_spi.setBitOrder(MSBFIRST);
            m_spi.setClockDivider(calculateSpiDivisor(MAX_SPI_INIT_CLK_HZ));
            m_spi.setDataMode(SPI_MODE3);
            pinMode(m_csPin, OUTPUT);

            m_shortPeriod = m_board->getClockSpeed() / 1000000 * SHORT_THRESHOLD;

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

            setClockDivider(calculateSpiDivisor(MAX_SPI_CLK_HZ));
            delayMicroseconds(1);

            setClockDivider(calculateSpiDivisor(MAX_SPI_INIT_CLK_HZ));

            // Accel and Gyro DLPF Setting
            writeRegister(REG_CONFIG, 0); // no gyro DLPF
            delayMicroseconds(1);

            setClockDivider(calculateSpiDivisor(MAX_SPI_CLK_HZ));
        }

        virtual int16_t readRawGyro(uint8_t k) override
        {
            return getValue(9 + k*2);
        }

    public:

        /*virtual*/ int16_t readRawAccel(uint8_t k) // override
        {
            return getValue(1 + k*2);
        }

        Bmi270(
                const uint8_t mosiPin,
                const uint8_t misoPin,
                const uint8_t sclkPin,
                const uint8_t csPin,
                const rotateFun_t rotateFun,
                const uint8_t sampleRateDivisor = 19,
                const gyroScale_e gyroScale = GYRO_2000DPS,
                const accelScale_e accelScale = ACCEL_2G)
            : SoftQuatImu(rotateFun, gyroScaleToInt(gyroScale) / 32768.)
        {
            m_mosiPin = mosiPin;
            m_misoPin = misoPin;
            m_sclkPin = sclkPin;
            m_csPin = csPin;
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
            int32_t gyroLastPeriod = intcmp(nowCycles, prevTime);

            // This detects the short (~79us) EXTI interval of an MPU6xxx gyro
            if ((m_shortPeriod == 0) || (gyroLastPeriod < m_shortPeriod)) {

                m_gyroSyncTime = prevTime;
            }

            prevTime = nowCycles;

            RealImu::handleInterrupt();
        }

}; // class Bmi270
