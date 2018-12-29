/*
   stm32fboard.h : Board class for STM4F boards

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

#pragma once

#include <boards/realboard.hpp>
#include <boards/softquat.hpp>

#include <MPU6xx0.h>

// Cleanflight includes
extern "C" {

#include "platform.h"
#include "drivers/system.h"
#include "drivers/timer.h"
#include "drivers/time.h"
#include "drivers/pwm_output.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_usb_vcp.h"
#include "io/serial.h"
#include "target.h"

} // extern "C"

class Stm32FBoard : public hf::RealBoard, public hf::SoftwareQuaternionBoard  {

    private:

        MPU6xx0 * _mpu;

    protected: 

        Stm32FBoard(serialPort_t * serial0);

        void checkImuError(MPUIMU::Error_t errid);

        // Board class overrides
        virtual void     writeMotor(uint8_t index, float value) override;
        virtual void     setLed(bool isOn) override;
        virtual void     reboot(void) override;
        virtual uint8_t  serialNormalAvailable(void) override;
        virtual uint8_t  serialNormalRead(void) override;
        virtual void     serialNormalWrite(uint8_t c) override;
        virtual bool     getQuaternion(float quat[4]) override;
        virtual bool     getGyrometer(float gyroRates[3]) override;

}; // class Stm32FBoard
