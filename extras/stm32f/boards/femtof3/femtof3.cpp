/*
   femtof3.h : Board implmentation for Hyperion F3 Evo Brushed board

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

#include "femtof3.h"

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
#include "../../common/motors.h"

    static serialPort_t * _serial0;

    FemtoF3::FemtoF3(void)
    {
        initMotors();
        initUsb();
        initImu();

        RealBoard::init();
    }

    void FemtoF3::initImu(void)
    {
        spi_init(MPU6500_SPI_INSTANCE, IOGetByTag(IO_TAG(MPU6500_CS_PIN)));

        _imu = new MPU6500(MPUIMU::AFS_2G, MPUIMU::GFS_250DPS);

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

    void FemtoF3::initUsb(void)
    {
        _serial0 = usbVcpOpen();
    }

    void FemtoF3::initMotors(void)
    {
        brushless_motors_init(0, 1, 2, 3);
    }

    void Stm32FBoard::writeMotor(uint8_t index, float value)
    {
        motor_write(index, value);
    }

    void Stm32FBoard::delaySeconds(float sec)
    {
        delay((uint16_t)(sec*1000));
    }

    void Stm32FBoard::setLed(bool isOn)
    {
        ledSet(0, isOn);
    }

    uint32_t Stm32FBoard::getMicroseconds(void)
    {
        return micros();
    }

    void Stm32FBoard::reboot(void)
    {
        systemResetToBootloader();
    }

    uint8_t Stm32FBoard::serialNormalAvailable(void)
    {
        return serialRxBytesWaiting(_serial0);
    }

    uint8_t Stm32FBoard::serialNormalRead(void)
    {
        return serialRead(_serial0);
    }

    void Stm32FBoard::serialNormalWrite(uint8_t c)
    {
        serialWrite(_serial0, c);
    }

    bool Stm32FBoard::getQuaternion(float quat[4])
    {
        return SoftwareQuaternionBoard::getQuaternion(quat, getTime());
    }

    bool Stm32FBoard::getGyrometer(float gyroRates[3])
    {
        return SoftwareQuaternionBoard::getGyrometer(gyroRates);
    }

    bool FemtoF3::imuRead(void)
    {
        if (_imu->checkNewData()) {  

            _imu->readAccelerometer(_ax, _ay, _az);
            _imu->readGyrometer(_gx, _gy, _gz);

            // Negate for IMU orientation
            _ay = -_ay;
            _gx = -_gx;
            _gz = -_gz;

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
