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

#include <constrain.h>
#include <imus/fusion/mpu6000.h>
#include <system.h>
#include <time.h>

#include <atomic.h>
#include <bus_spi.h>
#include <nvic.h>
#include <platform.h>
#include <systemdev.h>

static Mpu6000::gyroDev_t m_gyroDev;

void Mpu6000::interruptHandler(extiCallbackRec_t *cb)
{
    (void)cb;

    static uint32_t prevTime;

    // Ideally we'd use a time to capture such information, but unfortunately
    // the port used for EXTI interrupt does not have an associated timer
    uint32_t nowCycles = systemGetCycleCounter();
    int32_t gyroLastPeriod = cmpTimeCycles(nowCycles, prevTime);

    // This detects the short (~79us) EXTI interval of an MPU6xxx gyro
    if ((m_gyroDev.shortPeriod == 0) ||
            (gyroLastPeriod < m_gyroDev.shortPeriod)) {

        *m_gyroDev.syncTimePtr = prevTime + m_gyroDev.dmaMaxDuration;
    }

    prevTime = nowCycles;

    *m_gyroDev.interruptCountPtr = *m_gyroDev.interruptCountPtr  + 1;
}

// ----------------------------------------------------------------------------

bool Mpu6000::detectSPISensorsAndUpdateDetectionResult(
        const Mpu6000::gyroDeviceConfig_t *config)
{
   extDevice_t *dev = &m_gyroDev.dev;

    if (!config->csnTag || !spiSetBusInstance(dev, config->spiBus)) {
        return false;
    }

    dev->busType_u.spi.csnPin = IOGetByTag(config->csnTag);

    IOInit(dev->busType_u.spi.csnPin, OWNER_GYRO_CS, RESOURCE_INDEX(config->index));

    IOConfigGPIO(dev->busType_u.spi.csnPin, SPI_IO_CS_CFG);

    // Ensure device is disabled, important when two devices are on the same bus.
    IOHi(dev->busType_u.spi.csnPin); 

    spiSetClkDivisor(dev, spiCalculateDivider(MAX_SPI_INIT_CLK_HZ));

    // reset the device configuration
    spiWriteReg(dev, RA_PWR_MGMT_1, BIT_H_RESET);
    delay(100);  // datasheet specifies a 100ms delay after reset

    // reset the device signal paths
    spiWriteReg(dev, RA_SIGNAL_PATH_RESET, BIT_GYRO | BIT_ACC | BIT_TEMP);
    delay(100);  // datasheet specifies a 100ms delay after signal path reset

    spiSetClkDivisor(dev, spiCalculateDivider(MAX_SPI_CLK_HZ));

    busDeviceRegister(dev);

    return true;
}

bool Mpu6000::devGyroIsReady(void)
{
    uint16_t *gyroData = (uint16_t *)m_gyroDev.dev.rxBuf;

    // Ensure any prior DMA has completed before continuing
    spiWaitClaim(&m_gyroDev.dev);

    m_gyroDev.dev.txBuf[0] = Mpu6000::RA_GYRO_XOUT_H | 0x80;

    busSegment_t segments[] = {
        {NULL, NULL, 7, true, NULL},
        {NULL, NULL, 0, true, NULL},
    };
    segments[0].txData = m_gyroDev.dev.txBuf;
    segments[0].rxData = &m_gyroDev.dev.rxBuf[1];

    spiSequence(&m_gyroDev.dev, &segments[0]);

    // Wait for completion
    spiWait(&m_gyroDev.dev);

    m_gyroDev.adcRaw[0] = __builtin_bswap16(gyroData[1]);
    m_gyroDev.adcRaw[1] = __builtin_bswap16(gyroData[2]);
    m_gyroDev.adcRaw[2] = __builtin_bswap16(gyroData[3]);

    return true;
}

bool Mpu6000::mpuDetect(const Mpu6000::gyroDeviceConfig_t *config)
{
    static busDevice_t bus;
    m_gyroDev.dev.bus = &bus;

    // MPU datasheet specifies 30ms.
    delay(35);

    if (config->busType == BUS_TYPE_NONE) {
        return false;
    }

    if (config->busType == BUS_TYPE_GYRO_AUTO) {
        m_gyroDev.dev.bus->busType = BUS_TYPE_I2C;
    } else {
        m_gyroDev.dev.bus->busType = config->busType;
    }

    m_gyroDev.dev.bus->busType = BUS_TYPE_SPI;

    return detectSPISensorsAndUpdateDetectionResult(config);
}


void Mpu6000::devInit(uint32_t * gyroSyncTimePtr, uint32_t * gyroInterruptCountPtr)
{
    m_gyroDev.syncTimePtr = gyroSyncTimePtr;
    m_gyroDev.interruptCountPtr = gyroInterruptCountPtr;

    static gyroDeviceConfig_t gyroDeviceConfig; 

    gyroDeviceConfig.busType = BUS_TYPE_SPI;
    gyroDeviceConfig.spiBus = 1;
    gyroDeviceConfig.csnTag = 20;
    gyroDeviceConfig.extiTag = 52;

    spiPreinitRegister(gyroDeviceConfig.csnTag, IOCFG_IPU, 1);

    mpuDetect(&gyroDeviceConfig);

    m_gyroDev.shortPeriod = systemClockMicrosToCycles(SHORT_THRESHOLD);

    // SPI DMA buffer required per device
    static uint8_t gyroBuf1[GYRO_BUF_SIZE];
    m_gyroDev.dev.txBuf = gyroBuf1;
    m_gyroDev.dev.rxBuf = &gyroBuf1[GYRO_BUF_SIZE / 2];

    const IO_t mpuIntIO = IOGetByTag(gyroDeviceConfig.extiTag);

    IOInit(mpuIntIO, OWNER_GYRO_EXTI, 0);
    EXTIHandlerInit(&m_gyroDev.exti, interruptHandler);
    EXTIConfig(mpuIntIO, &m_gyroDev.exti, NVIC_PRIO_MPU_INT_EXTI, IOCFG_IN_FLOATING,
            BETAFLIGHT_EXTI_TRIGGER_RISING);
    EXTIEnable(mpuIntIO, true);

    spiSetClkDivisor(&m_gyroDev.dev, spiCalculateDivider(MAX_SPI_INIT_CLK_HZ));

    // Clock Source PPL with Z axis gyro reference
    spiWriteReg(&m_gyroDev.dev, RA_PWR_MGMT_1, CLK_SEL_PLLGYROZ);
    delayMicroseconds(15);

    // Disable Primary I2C Interface
    spiWriteReg(&m_gyroDev.dev, RA_USER_CTRL, BIT_I2C_IF_DIS);
    delayMicroseconds(15);

    spiWriteReg(&m_gyroDev.dev, RA_PWR_MGMT_2, 0x00);
    delayMicroseconds(15);

    // Accel Sample Rate 1kHz
    // Gyroscope Output Rate =  1kHz when the DLPF is enabled
    spiWriteReg(&m_gyroDev.dev, RA_SMPLRT_DIV, 0);
    delayMicroseconds(15);

    // Gyro +/- 2000 DPS Full Scale
    spiWriteReg(&m_gyroDev.dev, RA_GYRO_CONFIG, INV_FSR_2000DPS << 3);
    delayMicroseconds(15);

    // Accel +/- 16 G Full Scale
    spiWriteReg(&m_gyroDev.dev, RA_ACCEL_CONFIG, INV_FSR_16G << 3);
    delayMicroseconds(15);

    // INT_ANYRD_2CLEAR
    spiWriteReg(&m_gyroDev.dev, RA_INT_PIN_CFG,
            0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0);

    delayMicroseconds(15);

    spiWriteReg(&m_gyroDev.dev, RA_INT_ENABLE, RF_DATA_RDY_EN);
    delayMicroseconds(15);

    spiSetClkDivisor(&m_gyroDev.dev, spiCalculateDivider(MAX_SPI_CLK_HZ));
    delayMicroseconds(1);

    spiSetClkDivisor(&m_gyroDev.dev, spiCalculateDivider(MAX_SPI_INIT_CLK_HZ));

    // Accel and Gyro DLPF Setting
    spiWriteReg(&m_gyroDev.dev, CONFIG, 0); // no gyro DLPF
    delayMicroseconds(1);

    spiSetClkDivisor(&m_gyroDev.dev, spiCalculateDivider(MAX_SPI_CLK_HZ));

    if (((int8_t)m_gyroDev.adcRaw[1]) == -1 && ((int8_t)m_gyroDev.adcRaw[0]) == -1) {
        systemFailureMode(FAILURE_GYRO_INIT_FAILED);
    }
}

int16_t Mpu6000::devReadRawGyro(uint8_t k)
{
    return m_gyroDev.adcRaw[k];
}
