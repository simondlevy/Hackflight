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

        int16_t m_adcRaw[3];                          
        uint8_t m_csPin;
        uint8_t m_extiPin;

        virtual bool devGyroIsReady(void) override;

        virtual void devInit(
                uint32_t * gyroSyncTimePtr, uint32_t * gyroInterruptCountPtr) override;

        virtual int16_t devReadRawGyro(uint8_t k) override;

        static uint16_t calculateSpiDivisor(const uint32_t freq);

    public:

        // Shared with interrupt handler routine
        typedef struct {

            void * dev;

            int32_t           dmaMaxDuration;
            int32_t           shortPeriod;
            uint32_t *        syncTimePtr;
            uint32_t *        interruptCountPtr;

        } gyroDev_t;

        Mpu6000(uint8_t csPin, uint16_t gyroScale);

        void handleInterrupt(void);

}; // class Mpu6000
