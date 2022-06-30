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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <accel.h>
#include <maths.h>
#include <system.h>
#include <time.h>
#include "macros.h"

#include "imu_mpu.h"
#include "atomic.h"
#include "bus.h"
#include "bus_spi.h"
#include "exti.h"
#include "io.h"
#include "nvic.h"
#include "platform.h"
#include "systemdev.h"

#define MPU_ADDRESS        0x68
#define MPU_INQUIRY_MASK   0x7E

// Need to see at least this many interrupts during initialisation to confirm EXTI connectivity
#define GYRO_EXTI_DETECT_THRESHOLD 1000

// The gyro buffer is split 50/50, the first half for the transmit buffer, the
// second half for the receive buffer This buffer is large enough for the gyros
// currently supported in imu_mpu.c but should be reviewed id other gyro
// types are supported with SPI DMA.
#define GYRO_BUF_SIZE 32

busStatus_e mpuIntcallback(uint32_t arg)
{
    gyroDev_t *gyro = (gyroDev_t *)arg;
    int32_t gyroDmaDuration = cmpTimeCycles(systemGetCycleCounter(), gyro->gyroLastEXTI);

    if (gyroDmaDuration > gyro->gyroDmaMaxDuration) {
        gyro->gyroDmaMaxDuration = gyroDmaDuration;
    }

    gyro->dataReady = true;

    return BUS_READY;
}

static void mpuIntExtiHandler(extiCallbackRec_t *cb)
{
    gyroDev_t *gyroDev = container_of(cb, gyroDev_t, exti);

    // Ideally we'd use a time to capture such information, but unfortunately
    // the port used for EXTI interrupt does not have an associated timer
    uint32_t nowCycles = systemGetCycleCounter();
    int32_t gyroLastPeriod = cmpTimeCycles(nowCycles, gyroDev->gyroLastEXTI);
    // This detects the short (~79us) EXTI interval of an MPU6xxx gyro
    if ((gyroDev->gyroShortPeriod == 0) || (gyroLastPeriod < gyroDev->gyroShortPeriod)) {
        gyroDev->gyroSyncEXTI = gyroDev->gyroLastEXTI + gyroDev->gyroDmaMaxDuration;
    }
    gyroDev->gyroLastEXTI = nowCycles;

    gyroDev->detectedEXTI++;
}

static void mpuIntExtiInit(gyroDev_t *gyro)
{
    if (gyro->mpuIntExtiTag == IO_TAG_NONE) {
        return;
    }

    const IO_t mpuIntIO = IOGetByTag(gyro->mpuIntExtiTag);

    IOInit(mpuIntIO, OWNER_GYRO_EXTI, 0);
    EXTIHandlerInit(&gyro->exti, mpuIntExtiHandler);
    EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, IOCFG_IN_FLOATING,
            BETAFLIGHT_EXTI_TRIGGER_RISING);
    EXTIEnable(mpuIntIO, true);
}

bool mpuAccRead(accDev_t *acc)
{
    uint8_t data[6];

    const bool ack = busReadRegisterBuffer(&acc->gyro->dev, MPU_RA_ACCEL_XOUT_H, data, 6);
    if (!ack) {
        return false;
    }

    acc->ADCRaw[0] = (int16_t)((data[0] << 8) | data[1]);
    acc->ADCRaw[1] = (int16_t)((data[2] << 8) | data[3]);
    acc->ADCRaw[2] = (int16_t)((data[4] << 8) | data[5]);

    return true;
}

bool mpuGyroRead(gyroDev_t *gyro)
{
    uint8_t data[6];

    const bool ack = busReadRegisterBuffer(&gyro->dev, MPU_RA_GYRO_XOUT_H, data, 6);
    if (!ack) {
        return false;
    }

    gyro->adcRaw[0] = (int16_t)((data[0] << 8) | data[1]);
    gyro->adcRaw[1] = (int16_t)((data[2] << 8) | data[3]);
    gyro->adcRaw[2] = (int16_t)((data[4] << 8) | data[5]);

    return true;
}

bool mpuAccReadSPI(accDev_t *acc)
{
    // Ensure any prior DMA has completed before continuing
    spiWaitClaim(&acc->gyro->dev);

    acc->gyro->dev.txBuf[0] = MPU_RA_ACCEL_XOUT_H | 0x80;

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

bool mpuGyroReadSPI(gyroDev_t *gyro)
{
    uint16_t *gyroData = (uint16_t *)gyro->dev.rxBuf;

    // Ensure any prior DMA has completed before continuing
    spiWaitClaim(&gyro->dev);

    gyro->dev.txBuf[0] = MPU_RA_GYRO_XOUT_H | 0x80;

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

static bool detectSPISensorsAndUpdateDetectionResult(gyroDev_t *gyro,
        const gyroDeviceConfig_t *config)
{
    if (!config->csnTag || !spiSetBusInstance(&gyro->dev, config->spiBus)) {
        return false;
    }

    gyro->dev.busType_u.spi.csnPin = IOGetByTag(config->csnTag);

    IOInit(gyro->dev.busType_u.spi.csnPin, OWNER_GYRO_CS, RESOURCE_INDEX(config->index));
    IOConfigGPIO(gyro->dev.busType_u.spi.csnPin, SPI_IO_CS_CFG);

    // Ensure device is disabled, important when two devices are on the same bus.
    IOHi(gyro->dev.busType_u.spi.csnPin); 

    uint8_t sensor = MPU_NONE;

    // It is hard to use hardware to optimize the detection loop here,
    // as hardware type and detection function name doesn't match.
    // May need a bitmap of hardware to detection function to do it right?
    sensor = mpuBusDetect(&gyro->dev);
    if (sensor != MPU_NONE) {
        gyro->mpuDetectionResult.sensor = sensor;
        busDeviceRegister(&gyro->dev);
        return true;
    }

    // Detection failed, disable CS pin again
    spiPreinitByTag(config->csnTag);

    return false;
}

void mpuPreInit(gyroDeviceConfig_t * config)
{
    spiPreinitRegister(config->csnTag, IOCFG_IPU, 1);
}

bool mpuDetect(gyroDev_t *gyro, const gyroDeviceConfig_t *config)
{
    static busDevice_t bus;
    gyro->dev.bus = &bus;

    // MPU datasheet specifies 30ms.
    delay(35);

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

void mpuGyroInit(gyroDev_t *gyro)
{
    mpuIntExtiInit(gyro);
}

uint8_t mpuGyroDLPF(gyroDev_t *gyro)
{
    (void)gyro;
    return 0;
}

uint8_t mpuGyroReadRegister(const extDevice_t *dev, uint8_t reg)
{
    uint8_t data;
    const bool ack = busReadRegisterBuffer(dev, reg, &data, 1);
    if (ack) {
        return data;
    } else {
        return 0;
    }

}


// Glue code ------------------------------------------------------------------

static accDev_t accelDev;
static gyroDev_t gyroDev;

static const mpuDetectionResult_t *gyroMpuDetectionResult(void)
{
    return &gyroDev.mpuDetectionResult;
}

void accelInit(void)
{
    accelDev.gyro = &gyroDev;
    accelDev.mpuDetectionResult = *gyroMpuDetectionResult();
    accelDev.acc_high_fsr = false;
    mpuBusAccDetect(&accelDev);
}

bool accelIsReady(void)
{
    return accelDev.readFn(&accelDev);
}

float accelRead(uint8_t k) 
{
    return accelDev.ADCRaw[k];
}


void gyroDevInit(void)
{
    gyroDeviceConfig_t gyroDeviceConfig;

    mpuPreInit(&gyroDeviceConfig);

    gyroDeviceConfig.index = 0;
    gyroDeviceConfig.busType = 2;
    gyroDeviceConfig.spiBus = 1;
    gyroDeviceConfig.csnTag = 20;
    gyroDeviceConfig.i2cBus = 0;
    gyroDeviceConfig.i2cAddress = 0;
    gyroDeviceConfig.extiTag = 52;

    mpuDetect(&gyroDev, &gyroDeviceConfig);
    mpuBusGyroDetect(&gyroDev);

    // SPI DMA buffer required per device
    static uint8_t gyroBuf1[GYRO_BUF_SIZE];
    gyroDev.dev.txBuf = gyroBuf1;
    gyroDev.dev.rxBuf = &gyroBuf1[GYRO_BUF_SIZE / 2];

    gyroDev.mpuIntExtiTag = gyroDeviceConfig.extiTag;

    gyroDev.gyroSampleRateHz = gyroSetSampleRate(&gyroDev);
    gyroDev.initFn(&gyroDev);
}

float gyroScale(void)
{
    return gyroDev.scale;
}

uint32_t gyroInterruptCount(void)
{
    return gyroDev.detectedEXTI;
}

bool  gyroIsReady(void)
{
    bool ready = gyroDev.readFn(&gyroDev);

    if (ready) {
        gyroDev.dataReady = false;
    }

    return ready;
}

uint32_t gyroSyncTime(void)
{
    return gyroDev.gyroSyncEXTI;
}

int16_t gyroReadRaw(uint8_t k)
{
    return gyroDev.adcRaw[k];
}

bool gyroSyncCheckUpdate(gyroDev_t *gyro)
{
    bool ret;
    if (gyro->dataReady) {
        ret = true;
        gyro->dataReady= false;
    } else {
        ret = false;
    }
    return ret;
}

uint16_t gyroSetSampleRate(gyroDev_t *gyro)
{
    gyro->accSampleRateHz = 1000;

    return 8000;
}

void imuInit(hackflight_t * hf, uint8_t interruptPin)
{
    (void)hf;
    (void)interruptPin;

    gyroDevInit();
    accelInit();
}
