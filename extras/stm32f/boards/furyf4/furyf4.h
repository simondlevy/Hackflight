/*
   furyf4.h : Board class for FuryF4

   Copyright (C) 2019 Simon D. Levy 

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
#include <boards/softquat2.hpp>

#include "MPU6000.h"

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


class FuryF4 : public hf::RealBoard, public hf::SoftwareQuaternionBoard2 {

    private:

        MPU6000 * _imu;

    	void checkImuError(MPU6000::Error_t errid);

        // debugging
        int16_t _accx, _accy, _accz;

    protected: 

        FuryF4(serialPort_t * serial0);

        // Board class overrides
        virtual void     writeMotor(uint8_t index, float value) override;
        virtual void     setLed(bool isOn) override;
        virtual void     reboot(void) override;
        virtual uint8_t  serialNormalAvailable(void) override;
        virtual uint8_t  serialNormalRead(void) override;
        virtual void     serialNormalWrite(uint8_t c) override;
        virtual bool     getQuaternion(float & qw, float & qx, float & qy, float & qz) override;
        virtual bool     getGyrometer(float & gx, float & gy, float & gz) override;

        virtual void     adHocDebug(void) override;

        // SoftwareQuaternionBoard class overrides
        virtual bool imuReady(void) override;
        virtual void imuReadAccelGyro(void) override;

    public:

        FuryF4(void);

}; // class FuryF4
