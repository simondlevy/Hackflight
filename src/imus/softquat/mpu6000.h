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

#include <imus/softquat.h>

#include <exti.h>
#include <spi.h>

class Mpu6000 : public SoftQuatImu {

    private:

        // RF = Register Flag
        static const uint8_t RF_DATA_RDY_EN = 1 << 0;

        static const uint8_t CONFIG = 0x1A;

        // 1 MHz max SPI frequency for initialisation
        static const uint32_t MAX_SPI_INIT_CLK_HZ = 1000000;

        // 20 MHz max SPI frequency
        static const uint32_t MAX_SPI_CLK_HZ = 20000000;

        // Any interrupt interval less than this will be recognised as the
        // short interval of ~79us
        static const uint8_t SHORT_THRESHOLD = 82 ;

        // RA = Register Address;
        static const uint8_t RA_SMPLRT_DIV         = 0x19;
        static const uint8_t RA_GYRO_CONFIG        = 0x1B;
        static const uint8_t RA_ACCEL_CONFIG       = 0x1C;
        static const uint8_t RA_INT_PIN_CFG        = 0x37;
        static const uint8_t RA_INT_ENABLE         = 0x38;
        static const uint8_t RA_GYRO_XOUT_H        = 0x43;
        static const uint8_t RA_USER_CTRL          = 0x6A;
        static const uint8_t RA_PWR_MGMT_1         = 0x6B;
        static const uint8_t RA_PWR_MGMT_2         = 0x6C;
        static const uint8_t RA_SIGNAL_PATH_RESET  = 0x68;

        // The gyro buffer is split 50/50, the first half for the transmit
        // buffer, the second half for the receive buffer This buffer is large
        // enough for the gyros currently supported in imu_mpu.c but should be
        // reviewed id other gyro types are supported with SPI DMA.
        static const uint8_t GYRO_BUF_SIZE = 32;

        static const uint8_t CLK_SEL_PLLGYROZ = 0x03;

        static const uint8_t BIT_TEMP       = 0x01;
        static const uint8_t BIT_ACC        = 0x02;
        static const uint8_t BIT_GYRO       = 0x04;
        static const uint8_t BIT_I2C_IF_DIS = 0x10;
        static const uint8_t BIT_H_RESET    = 0x80;

        enum gyro_fsr_e {
            INV_FSR_250DPS = 0,
            INV_FSR_500DPS,
            INV_FSR_1000DPS,
            INV_FSR_2000DPS,
            NUM_GYRO_FSR
        };

        enum accel_fsr_e {
            INV_FSR_2G = 0,
            INV_FSR_4G,
            INV_FSR_8G,
            INV_FSR_16G,
            NUM_ACCEL_FSR
        };

        int16_t    m_adcRaw[3];                          
        uint8_t    m_csPin;
        void *     m_spi;
        int32_t    m_dmaMaxDuration;
        int32_t    m_shortPeriod;
        uint32_t * m_syncTimePtr;
        uint32_t * m_interruptCountPtr;

        static uint16_t calculateSpiDivisor(const uint32_t freq)
        {
            uint32_t clk = systemGetClockSpeed() / 2;

            uint16_t divisor = 2;

            clk >>= 1;

            for (; (clk > freq) && (divisor < 256); divisor <<= 1, clk >>= 1);

            return divisor;
        }

        bool devGyroIsReady(void) override
        {
            uint8_t * rxBuf = spiGetRxBuf(m_spi);
            uint8_t * txBuf = spiGetTxBuf(m_spi);

            uint16_t * gyroData = (uint16_t *)rxBuf;

            // Ensure any prior DMA has completed before continuing
            spiWait(m_spi);

            txBuf[0] = Mpu6000::RA_GYRO_XOUT_H | 0x80;

            busSegment_t segments[] =
            { {txBuf, &rxBuf[1], 7, true}, {NULL, NULL, 0, true} };

            spiSequence(m_spi, segments);

            // Wait for completion
            spiWait(m_spi);

            m_adcRaw[0] = __builtin_bswap16(gyroData[1]);
            m_adcRaw[1] = __builtin_bswap16(gyroData[2]);
            m_adcRaw[2] = __builtin_bswap16(gyroData[3]);

            return true;
        }

        void devInit(uint32_t * gyroSyncTimePtr, uint32_t * gyroInterruptCountPtr) override
        {
            m_syncTimePtr = gyroSyncTimePtr;
            m_interruptCountPtr = gyroInterruptCountPtr;

            void *dev = m_spi;

            spiSetClkDivisor(dev, calculateSpiDivisor(MAX_SPI_INIT_CLK_HZ));

            // reset the device configuration
            spiWriteReg(dev, RA_PWR_MGMT_1, BIT_H_RESET);
            delay(100);  // datasheet specifies a 100ms delay after reset

            // reset the device signal paths
            spiWriteReg(dev, RA_SIGNAL_PATH_RESET, BIT_GYRO | BIT_ACC | BIT_TEMP);
            delay(100);  // datasheet specifies a 100ms delay after signal path reset

            spiSetClkDivisor(dev, calculateSpiDivisor(MAX_SPI_CLK_HZ));

            m_shortPeriod = systemClockMicrosToCycles(SHORT_THRESHOLD);

            // SPI DMA buffer required per device
            static uint8_t gyroBuf1[GYRO_BUF_SIZE];
            spiSetRxBuf(dev, &gyroBuf1[GYRO_BUF_SIZE / 2]);
            spiSetTxBuf(dev, gyroBuf1);

            spiSetClkDivisor(m_spi, calculateSpiDivisor(MAX_SPI_INIT_CLK_HZ));

            // Clock Source PPL with Z axis gyro reference
            spiWriteReg(m_spi, RA_PWR_MGMT_1, CLK_SEL_PLLGYROZ);
            delayMicroseconds(15);

            // Disable Primary I2C Interface
            spiWriteReg(m_spi, RA_USER_CTRL, BIT_I2C_IF_DIS);
            delayMicroseconds(15);

            spiWriteReg(m_spi, RA_PWR_MGMT_2, 0x00);
            delayMicroseconds(15);

            // Accel Sample Rate 1kHz
            // Gyroscope Output Rate =  1kHz when the DLPF is enabled
            spiWriteReg(m_spi, RA_SMPLRT_DIV, 0);
            delayMicroseconds(15);

            // Gyro +/- 2000 DPS Full Scale
            spiWriteReg(m_spi, RA_GYRO_CONFIG, INV_FSR_2000DPS << 3);
            delayMicroseconds(15);

            // Accel +/- 16 G Full Scale
            spiWriteReg(m_spi, RA_ACCEL_CONFIG, INV_FSR_16G << 3);
            delayMicroseconds(15);

            // INT_ANYRD_2CLEAR
            spiWriteReg(m_spi, RA_INT_PIN_CFG,
                    0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0);

            delayMicroseconds(15);

            spiWriteReg(m_spi, RA_INT_ENABLE, RF_DATA_RDY_EN);
            delayMicroseconds(15);

            spiSetClkDivisor(m_spi, calculateSpiDivisor(MAX_SPI_CLK_HZ));
            delayMicroseconds(1);

            spiSetClkDivisor(m_spi, calculateSpiDivisor(MAX_SPI_INIT_CLK_HZ));

            // Accel and Gyro DLPF Setting
            spiWriteReg(m_spi, CONFIG, 0); // no gyro DLPF
            delayMicroseconds(1);

            spiSetClkDivisor(m_spi, calculateSpiDivisor(MAX_SPI_CLK_HZ));
        }

        int16_t devReadRawGyro(uint8_t k) override
        {
            return m_adcRaw[k];
        }

    public:

        Mpu6000(uint8_t csPin, uint16_t gyroScale)
            : SoftQuatImu(gyroScale)
        {
            m_spi = spiGetInstance(csPin);
            m_csPin = csPin;
        }

        void handleInterrupt(void)
        {
            static uint32_t prevTime;

            // Ideally we'd use a time to capture such information, but unfortunately
            // the port used for EXTI interrupt does not have an associated timer
            uint32_t nowCycles = systemGetCycleCounter();
            int32_t gyroLastPeriod = cmpTimeCycles(nowCycles, prevTime);

            // This detects the short (~79us) EXTI interval of an MPU6xxx gyro
            if ((m_shortPeriod == 0) ||
                    (gyroLastPeriod < m_shortPeriod)) {

                *m_syncTimePtr = prevTime + m_dmaMaxDuration;
            }

            prevTime = nowCycles;

            *m_interruptCountPtr += 1;
        }


}; // class Mpu6000
