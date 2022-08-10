/*
   Copyright (c) 2022 Simon D. Levy

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

#pragma once

#include "hackflight.h"

void hackflightInitFull(
        hackflight_t * hf,
        rx_dev_funs_t * rxDeviceFuns,
        serialPortIdentifier_e rxDevPort,
        anglePidConstants_t * anglePidConstants,
        mixer_t mixer,
        void * motorDevice,
        uint8_t imuInterruptPin,
        imu_align_fun imuAlign,
        uint8_t ledPin);

void hackflightStep(hackflight_t * hackflight);

class Hackflight : HackflightCore {

    private:

        // Scheduling constants

        static const uint32_t RX_TASK_RATE       = 33;
        static const uint32_t ATTITUDE_TASK_RATE = 100;

        // Instance variable

        arming_t         m_arming;
        task_t           m_attitudeTask;
        gyro_t           m_gyro;
        imu_align_fun    m_imuAlignFun;
        imu_fusion_t     m_imuFusionPrev;
        float            m_maxArmingAngle;
        void *           m_motorDevice;
        float            m_mspMotors[4];
        task_t           m_mspTask;
        rx_t             m_rx;
        task_t           m_rxTask;
        rx_axes_t        m_rxAxes;
        scheduler_t      m_scheduler;
        task_t           m_sensorTasks[10];
        uint8_t          m_sensorTaskCount;

};
