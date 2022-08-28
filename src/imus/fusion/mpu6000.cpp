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
#include <imus/fusion/mpudev.h>
#include <system.h>
#include <time.h>

#include <atomic.h>
#include <bus.h>
#include <bus_spi.h>
#include <exti.h>
#include <io.h>
#include <nvic.h>
#include <platform.h>
#include <systemdev.h>

static void mpuIntExtiHandler(extiCallbackRec_t *cb)
{
    gyroDev_t *gyroDev = gyroContainerOf(cb);

    // Ideally we'd use a time to capture such information, but unfortunately
    // the port used for EXTI interrupt does not have an associated timer
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

// ----------------------------------------------------------------------------

mpuSensor_e mpuBusDetect(const extDevice_t *dev)
{

    spiSetClkDivisor(dev, spiCalculateDivider(Mpu6000Imu::MAX_SPI_INIT_CLK_HZ));

    // reset the device configuration
    spiWriteReg(dev, Mpu6000Imu::RA_PWR_MGMT_1, Mpu6000Imu::BIT_H_RESET);
    delay(100);  // datasheet specifies a 100ms delay after reset

    // reset the device signal paths
    spiWriteReg(dev, Mpu6000Imu::RA_SIGNAL_PATH_RESET,
            Mpu6000Imu::BIT_GYRO | Mpu6000Imu::BIT_ACC | Mpu6000Imu::BIT_TEMP);
    delay(100);  // datasheet specifies a 100ms delay after signal path reset


    const uint8_t whoAmI = spiReadRegMsk(dev, Mpu6000Imu::RA_WHO_AM_I);

    // Ensure CS high time is met which is violated on H7 without this delay
    delayMicroseconds(1); 
    mpuSensor_e detectedSensor = MPU_NONE;

    if (whoAmI == Mpu6000Imu::MPU6000_WHO_AM_I_CONST) {
        const uint8_t productID = spiReadRegMsk(dev, Mpu6000Imu::RA_PRODUCT_ID);

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

    gyro->readFn = Mpu6000Imu::gyroReadSPI;
    gyro->gyroShortPeriod = systemClockMicrosToCycles(Mpu6000Imu::SHORT_THRESHOLD);
    return true;
}


// ----------------------------------------------------------------------------

static gyroDev_t m_gyroDev;

bool Mpu6000Imu::gyroRead(void)
{
    uint8_t data[6];

    const bool ack = busReadRegisterBuffer(
            &m_gyroDev.dev, Mpu6000Imu::RA_GYRO_XOUT_H, data, 6);

    if (!ack) {
        return false;
    }

    m_gyroDev.adcRaw[0] = (int16_t)((data[0] << 8) | data[1]);
    m_gyroDev.adcRaw[1] = (int16_t)((data[2] << 8) | data[3]);
    m_gyroDev.adcRaw[2] = (int16_t)((data[4] << 8) | data[5]);

    return true;
}

bool Mpu6000Imu::gyroReadSPI()
{
    uint16_t *gyroData = (uint16_t *)m_gyroDev.dev.rxBuf;

    // Ensure any prior DMA has completed before continuing
    spiWaitClaim(&m_gyroDev.dev);

    m_gyroDev.dev.txBuf[0] = Mpu6000Imu::RA_GYRO_XOUT_H | 0x80;

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

typedef uint8_t (*gyroSpiDetectFn_t)(const extDevice_t *dev);

static bool detectSPISensorsAndUpdateDetectionResult(
        const Mpu6000Imu::gyroDeviceConfig_t *config)
{
    if (!config->csnTag || !spiSetBusInstance(&m_gyroDev.dev, config->spiBus)) {
        return false;
    }

    m_gyroDev.dev.busType_u.spi.csnPin = IOGetByTag(config->csnTag);

    IOInit(m_gyroDev.dev.busType_u.spi.csnPin, OWNER_GYRO_CS,
            RESOURCE_INDEX(config->index));

    IOConfigGPIO(m_gyroDev.dev.busType_u.spi.csnPin, SPI_IO_CS_CFG);

    // Ensure device is disabled, important when two devices are on the same bus.
    IOHi(m_gyroDev.dev.busType_u.spi.csnPin); 

    // It is hard to use hardware to optimize the detection loop here,
    // as hardware type and detection function name doesn't match.
    // May need a bitmap of hardware to detection function to do it right?
    auto sensor = mpuBusDetect(&m_gyroDev.dev);
    if (sensor != MPU_NONE) {
        m_gyroDev.mpuDetectionResult.sensor = sensor;
        busDeviceRegister(&m_gyroDev.dev);
        return true;
    }

    // Detection failed, disable CS pin again
    spiPreinitByTag(config->csnTag);

    return false;
}

static bool mpuDetect(const Mpu6000Imu::gyroDeviceConfig_t *config)
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

void Mpu6000Imu::gyroInit(void)
{
    if (m_gyroDev.mpuIntExtiTag == IO_TAG_NONE) {
        return;
    }

    const IO_t mpuIntIO = IOGetByTag(m_gyroDev.mpuIntExtiTag);

    IOInit(mpuIntIO, OWNER_GYRO_EXTI, 0);
    EXTIHandlerInit(&m_gyroDev.exti, mpuIntExtiHandler);
    EXTIConfig(mpuIntIO, &m_gyroDev.exti, NVIC_PRIO_MPU_INT_EXTI, IOCFG_IN_FLOATING,
            BETAFLIGHT_EXTI_TRIGGER_RISING);
    EXTIEnable(mpuIntIO, true);
}

// ----------------------------------------------------------------------------

uint32_t imuDevGyroInterruptCount(void)
{
    return m_gyroDev.detectedEXTI;
}

uint32_t imuDevGyroSyncTime(void)
{
    return m_gyroDev.gyroSyncEXTI;
}


// ----------------------------------------------------------------------------

bool Mpu6000Imu::devGyroIsReady(void)
{
    bool ready = m_gyroDev.readFn();

    if (ready) {
        m_gyroDev.dataReady = false;
    }

    return ready;
}

void Mpu6000Imu::devInit(uint8_t interruptPin)
{
    (void)interruptPin;

    static Mpu6000Imu::gyroDeviceConfig_t gyroDeviceConfig; 

    gyroDeviceConfig.busType = BUS_TYPE_SPI; // XXX pass from subclass
    gyroDeviceConfig.spiBus = 1;
    gyroDeviceConfig.csnTag = 20;
    gyroDeviceConfig.extiTag = 52;

    spiPreinitRegister(gyroDeviceConfig.csnTag, IOCFG_IPU, 1);

    mpuDetect(&gyroDeviceConfig);
    mpuBusGyroDetect(&m_gyroDev);

    // SPI DMA buffer required per device
    static uint8_t gyroBuf1[Mpu6000Imu::GYRO_BUF_SIZE];
    m_gyroDev.dev.txBuf = gyroBuf1;
    m_gyroDev.dev.rxBuf = &gyroBuf1[Mpu6000Imu::GYRO_BUF_SIZE / 2];

    m_gyroDev.mpuIntExtiTag = gyroDeviceConfig.extiTag;

    gyroInit();

    spiSetClkDivisor(&m_gyroDev.dev, spiCalculateDivider(Mpu6000Imu::MAX_SPI_INIT_CLK_HZ));

    // Device was already reset during detection so proceed with configuration

    // Clock Source PPL with Z axis gyro reference
    spiWriteReg(&m_gyroDev.dev, Mpu6000Imu::RA_PWR_MGMT_1, Mpu6000Imu::CLK_SEL_PLLGYROZ);
    delayMicroseconds(15);

    // Disable Primary I2C Interface
    spiWriteReg(&m_gyroDev.dev, Mpu6000Imu::RA_USER_CTRL, Mpu6000Imu::BIT_I2C_IF_DIS);
    delayMicroseconds(15);

    spiWriteReg(&m_gyroDev.dev, Mpu6000Imu::RA_PWR_MGMT_2, 0x00);
    delayMicroseconds(15);

    // Accel Sample Rate 1kHz
    // Gyroscope Output Rate =  1kHz when the DLPF is enabled
    spiWriteReg(&m_gyroDev.dev, Mpu6000Imu::RA_SMPLRT_DIV, 0);
    delayMicroseconds(15);

    // Gyro +/- 2000 DPS Full Scale
    spiWriteReg(&m_gyroDev.dev, Mpu6000Imu::RA_GYRO_CONFIG, Mpu6000Imu::INV_FSR_2000DPS << 3);
    delayMicroseconds(15);

    // Accel +/- 16 G Full Scale
    spiWriteReg(&m_gyroDev.dev, Mpu6000Imu::RA_ACCEL_CONFIG, Mpu6000Imu::INV_FSR_16G << 3);
    delayMicroseconds(15);

    spiWriteReg(&m_gyroDev.dev, Mpu6000Imu::RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0);  // INT_ANYRD_2CLEAR
    delayMicroseconds(15);

    spiWriteReg(&m_gyroDev.dev, Mpu6000Imu::RA_INT_ENABLE, Mpu6000Imu::RF_DATA_RDY_EN);
    delayMicroseconds(15);

    spiSetClkDivisor(&m_gyroDev.dev, spiCalculateDivider(Mpu6000Imu::MAX_SPI_CLK_HZ));
    delayMicroseconds(1);

    spiSetClkDivisor(&m_gyroDev.dev, spiCalculateDivider(Mpu6000Imu::MAX_SPI_INIT_CLK_HZ));

    // Accel and Gyro DLPF Setting
    spiWriteReg(&m_gyroDev.dev, Mpu6000Imu::CONFIG, 0); // no gyro DLPF
    delayMicroseconds(1);

    spiSetClkDivisor(&m_gyroDev.dev, spiCalculateDivider(Mpu6000Imu::MAX_SPI_CLK_HZ));

    Mpu6000Imu::gyroRead();

    if (((int8_t)m_gyroDev.adcRaw[1]) == -1 && ((int8_t)m_gyroDev.adcRaw[0]) == -1) {
        systemFailureMode(FAILURE_GYRO_INIT_FAILED);
    }
}

int16_t Mpu6000Imu::devReadRawGyro(uint8_t k)
{
    return m_gyroDev.adcRaw[k];
}
