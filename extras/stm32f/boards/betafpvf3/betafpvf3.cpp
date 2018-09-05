/*
   betafpvf3.h : Board implmentation for BetaFPV F3 Brushed board

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

#include "betafpvf3.h"

static const uint16_t BRUSHED_PWM_RATE     = 32000;
static const uint16_t BRUSHED_IDLE_PULSE   = 0; 

static const float    MOTOR_MIN = 1000;
static const float    MOTOR_MAX = 2000;

// Here we put code that interacts with Cleanflight
extern "C" {

    // Cleanflight includes
#include "platform.h"
#include "drivers/system.h"
#include "drivers/timer.h"
#include "drivers/time.h"
#include "drivers/pwm_output.h"
#include "drivers/light_led.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_usb_vcp.h"
#include "io/serial.h"
#include "target.h"
#include "stm32f30x.h"

    // Hackflight includes
#include "../../common/spi.h"
#include "../../common/beeperled.h"

    static serialPort_t * _serial0;

    BetaFPVF3::BetaFPVF3(void)
    {
        // Set up the LED (uses the beeper for some reason)
        beeperLedInit();

        initMotors();
        initUsb();
        initImu();

        RealBoard::init();
    }

    void BetaFPVF3::initImu(void)
    {
        spi_init(MPU6000_SPI_INSTANCE, IOGetByTag(IO_TAG(MPU6000_CS_PIN)));

        _imu = new MPU6000(MPUIMU::AFS_2G, MPUIMU::GFS_250DPS);

        switch (_imu->begin()) {

            case MPUIMU::ERROR_IMU_ID:
                error("Bad device ID");
                break;
            case MPUIMU::ERROR_SELFTEST:
                error("Failed self-test");
                break;
            default:
                break;
        }
    }

    void BetaFPVF3::initUsb(void)
    {
        _serial0 = usbVcpOpen();
    }

    void BetaFPVF3::initMotors(void)
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

    void BetaFPVF3::writeMotor(uint8_t index, float value)
    {
        pwmWriteMotor(index, MOTOR_MIN + value*(MOTOR_MAX-MOTOR_MIN));
    }

    void BetaFPVF3::delaySeconds(float sec)
    {
        delay((uint16_t)(sec*1000));
    }

    void BetaFPVF3::setLed(bool isOn)
    {
        beeperLedSet(isOn);
    }

    uint32_t BetaFPVF3::getMicroseconds(void)
    {
        return micros();
    }

    void BetaFPVF3::reboot(void)
    {
        systemResetToBootloader();
    }

    uint8_t BetaFPVF3::serialAvailableBytes(void)
    {
        return serialRxBytesWaiting(_serial0);
    }

    uint8_t BetaFPVF3::serialReadByte(void)
    {
        return serialRead(_serial0);
    }

    void BetaFPVF3::serialWriteByte(uint8_t c)
    {
        serialWrite(_serial0, c);
    }

    bool BetaFPVF3::imuRead(void)
    {
        if (_imu->checkNewData()) {  

            _imu->readAccelerometer(_ay, _ax, _az);
            _imu->readGyrometer(_gy, _gx, _gz);

            _gx = -_gx;
            _gy = -_gy;
            _gz = -_gz;

            hf::Debug::printlnfloat(_gz);

            return true;
        }  

        return false;
    }

                    // ay ax gy gx
                    // +  +  +  + 
                    // +  +  +  - 
                    // +  +  -  + 
                    // +  +  -  - 
                    // +  -  +  + 
                    // +  -  +  - 
                    // +  -  -  + 
                    // +  -  -  - 
                    // -  +  +  + 
                    // -  +  +  - 
                    // -  +  -  + 
                    // -  +  -  - 
                    // -  -  +  + 
                    // -  -  +  - 
                    // -  -  -  + 
                    // -  -  -  - 

    void hf::Board::outbuf(char * buf)
    {
        for (char *p=buf; *p; p++)
            serialWrite(_serial0, *p);
    }


} // extern "C"
