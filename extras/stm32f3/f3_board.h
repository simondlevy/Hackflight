/*
   f3_board.h : Support for STM32F3 boards

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

#include <boards/software_quaternion.hpp>
#include <filters.hpp>

class F3Board : public hf::SoftwareQuaternionBoard {

    friend class hf::Board;

    private:

    // Global constants for 6 DoF quaternion filter
    const float GYRO_MEAS_ERROR = M_PI * (40.0f / 180.0f); // gyroscope measurement error in rads/s (start at 40 deg/s)
    const float GYRO_MEAS_DRIFT = M_PI * (0.0f  / 180.0f); // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
    const float BETA = sqrtf(3.0f / 4.0f) * GYRO_MEAS_ERROR;   // compute BETA
    const float ZETA = sqrt(3.0f / 4.0f) * GYRO_MEAS_DRIFT;  

    // Update quaternion after this number of gyro updates
    const uint8_t QUATERNION_DIVISOR = 5;

    // Quaternion support: even though MPU9250 has a magnetometer, we keep it simple for now by 
    // using a 6DOF fiter (accel, gyro)
    hf::MadgwickQuaternionFilter6DOF _quaternionFilter = hf::MadgwickQuaternionFilter6DOF(BETA, ZETA);
    uint8_t _quatCycleCount;
    float _ax=0,_ay=0,_az=0,_gx=0,_gy=0,_gz=0;

    void error(const char * errmsg);

    void imuInit(void);

    void usbInit(void);

    bool getImu(int16_t accelCount[3], int16_t gyroCount[3]);

    protected:

    virtual bool  getQuaternion(float quat[4]) override;

    virtual bool  getGyrometer(float gyroRates[3]) override;

    virtual void  writeMotor(uint8_t index, float value) override;

    virtual void delaySeconds(float sec) override;

    virtual void ledSet(bool is_on) override;

    virtual uint32_t getMicroseconds(void) override;

    virtual void reboot(void) override;

    static void outchar(char c);

    virtual uint8_t serialAvailableBytes(void) override;

    virtual uint8_t serialReadByte(void) override;

    virtual void    serialWriteByte(uint8_t c) override;

    public:

    F3Board(void);

}; // class F3Board
