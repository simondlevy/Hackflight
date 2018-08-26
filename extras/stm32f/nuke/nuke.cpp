/*
   nuke.cpp : Board implementation for Furious FPV Nuke

   Copyright (C) 2018 Simon D. Levy 

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

#include "nuke.h"

// Bits
#define BIT_SLEEP                   0x40
#define BIT_H_RESET                 0x80
#define BITS_CLKSEL                 0x07
#define MPU_CLK_SEL_PLLGYROX        0x01
#define MPU_CLK_SEL_PLLGYROZ        0x03
#define MPU_EXT_SYNC_GYROX          0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_2G                  0x00
#define BITS_FS_4G                  0x08
#define BITS_FS_8G                  0x10
#define BITS_FS_16G                 0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_256HZ         0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR        0x10
#define BIT_RAW_RDY_EN              0x01
#define BIT_I2C_IF_DIS              0x10
#define BIT_INT_STATUS_DATA         0x01
#define BIT_GYRO                    3
#define BIT_ACC                     2
#define BIT_TEMP                    1

static const uint16_t BRUSHED_PWM_RATE     = 32000;
static const uint16_t BRUSHED_IDLE_PULSE   = 0; 

static const float    MOTOR_MIN = 1000;
static const float    MOTOR_MAX = 2000;

// Here we put code that interacts with Cleanflight
extern "C" {

#include "platform.h"

#include "drivers/system.h"
#include "drivers/timer.h"
#include "drivers/time.h"
#include "drivers/pwm_output.h"
#include "drivers/light_led.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_usb_vcp.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "pg/bus_spi.h"

#include "io/serial.h"

#include "target.h"

#include "stm32f30x.h"

#include "drivers/accgyro/accgyro_mpu.h"

    static serialPort_t * _serial0;

    static busDevice_t _bus;

    Nuke::Nuke(void)
    {
        initMotors();
        initUsb();
        initImu();

        RealBoard::init();
    }

    void Nuke::initImu(void)
    {
        spiPinConfigure(spiPinConfig(0));
        spiPreInit();

        SPIDevice spiDevice = SPIINVALID;

        if (MPU6000_SPI_INSTANCE == SPI1) {
            spiDevice = SPIDEV_1;
        }

        else if (MPU6000_SPI_INSTANCE == SPI2) {
            spiDevice = SPIDEV_2;
        }

        else if (MPU6000_SPI_INSTANCE == SPI3) {
            spiDevice = SPIDEV_3;
        }

        spiInit(spiDevice);

        spiBusSetInstance(&_bus, MPU6000_SPI_INSTANCE);

        _bus.busdev_u.spi.csnPin = IOGetByTag(IO_TAG(MPU6000_CS_PIN));

        delaySeconds(.01);

        IOInit(_bus.busdev_u.spi.csnPin, OWNER_MPU_CS, 0);
        IOConfigGPIO(_bus.busdev_u.spi.csnPin, SPI_IO_CS_CFG);
        IOHi(_bus.busdev_u.spi.csnPin);

        spiSetDivisor(_bus.busdev_u.spi.instance, SPI_CLOCK_INITIALIZATON);

        delaySeconds(.01);

        _imu = new MPU6000(AFS_2G, GFS_250DPS);

        switch (_imu->begin()) {

            case MPU_ERROR_ID:
                error("Bad device ID");
                break;
            case MPU_ERROR_SELFTEST:
                error("Failed self-test");
                break;
            default:
                break;
        }

        delaySeconds(.01);

        // Device Reset
        spiBusWriteRegister(&_bus, MPU_RA_PWR_MGMT_1, BIT_H_RESET);
        delay(150);

        spiBusWriteRegister(&_bus, MPU_RA_SIGNAL_PATH_RESET, BIT_GYRO | BIT_ACC | BIT_TEMP);
        delay(150);

        // Clock Source PPL with Z axis gyro reference
        spiBusWriteRegister(&_bus, MPU_RA_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
        delayMicroseconds(15);

        // Disable Primary I2C Interface
        spiBusWriteRegister(&_bus, MPU_RA_USER_CTRL, BIT_I2C_IF_DIS);
        delayMicroseconds(15);

        spiBusWriteRegister(&_bus, MPU_RA_PWR_MGMT_2, 0x00);
        delayMicroseconds(15);

        // Accel Sample Rate 1kHz
        // Gyroscope Output Rate =  1kHz when the DLPF is enabled
        //spiBusWriteRegister(&_bus, MPU_RA_SMPLRT_DIV, gyro->mpuDividerDrops);
        //delayMicroseconds(15);

        // Gyro +/- 1000 DPS Full Scale
        spiBusWriteRegister(&_bus, MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3);
        delayMicroseconds(15);

        // Accel +/- 16 G Full Scale
        spiBusWriteRegister(&_bus, MPU_RA_ACCEL_CONFIG, INV_FSR_16G << 3);
        delayMicroseconds(15);

        //spiBusWriteRegister(&_bus, MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0);  // INT_ANYRD_2CLEAR
        delayMicroseconds(15);

        spiBusWriteRegister(&_bus, MPU_RA_INT_ENABLE, MPU_RF_DATA_RDY_EN);

        spiSetDivisor(_bus.busdev_u.spi.instance, SPI_CLOCK_FAST);
    }

    static void _writeRegister(uint8_t subAddress, uint8_t data)
    {
        spiBusWriteRegister(&_bus, subAddress, data);
    }

    static void _readRegisters(uint8_t subAddress, uint8_t count, uint8_t * dest)
    {  
        spiBusReadRegisterBuffer(&_bus, subAddress | 0x80, dest, count);
    }

    void Nuke::initUsb(void)
    {
        _serial0 = usbVcpOpen();
    }

    void Nuke::initMotors(void)
    {
        motorDevConfig_t dev;

        dev.motorPwmRate = BRUSHED_PWM_RATE;
        dev.motorPwmProtocol = PWM_TYPE_BRUSHED;
        dev.motorPwmInversion = false;
        dev.useUnsyncedPwm = true;
        dev.useBurstDshot = false;

        dev.ioTags[0] = timerioTagGetByUsage(TIM_USE_MOTOR, 0);
        dev.ioTags[1] = timerioTagGetByUsage(TIM_USE_MOTOR, 1);
        dev.ioTags[2] = timerioTagGetByUsage(TIM_USE_MOTOR, 2);
        dev.ioTags[3] = timerioTagGetByUsage(TIM_USE_MOTOR, 3);

        motorDevInit(&dev, BRUSHED_IDLE_PULSE, 4);

        pwmEnableMotors();
    }

    void Nuke::writeMotor(uint8_t index, float value)
    {
        pwmWriteMotor(index, MOTOR_MIN + value*(MOTOR_MAX-MOTOR_MIN));
    }

    void Nuke::delaySeconds(float sec)
    {
        delay((uint16_t)(sec*1000));
    }

    void Nuke::setLed(bool is_on)
    {
        ledSet(0, is_on);
    }

    uint32_t Nuke::getMicroseconds(void)
    {
        return micros();
    }

    void Nuke::reboot(void)
    {
        systemResetToBootloader();
    }

    uint8_t Nuke::serialAvailableBytes(void)
    {
        return serialRxBytesWaiting(_serial0);
    }

    uint8_t Nuke::serialReadByte(void)
    {
        return serialRead(_serial0);
    }

    void Nuke::serialWriteByte(uint8_t c)
    {
        serialWrite(_serial0, c);
    }

    bool Nuke::imuRead(void)
    {
        if (_imu->checkNewData()) {  

            // Note reversed X/Y order because of IMU rotation
            _imu->readAccelerometer(_ax, _ay, _az);


            hf::Debug::printf("%d\n", (int16_t)(1000*_ay));

            _imu->readGyrometer(_gx, _gy, _gz);

            _az = -_az;

            return true;

        }  

        return false;
    }

    void hf::Board::outbuf(char * buf)
    {
        for (char *p=buf; *p; p++)
            serialWrite(_serial0, *p);
    }

} // extern "C"

#include <CrossPlatformSPI.h>

void cpspi_writeRegister(uint8_t subAddress, uint8_t data)
{
    _writeRegister(subAddress, data);
}

void cpspi_readRegisters(uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
    _readRegisters(subAddress, count, dest);
}

