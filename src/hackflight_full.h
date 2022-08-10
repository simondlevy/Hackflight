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

#include "attitude_task.h"
#include "receiver_task.h"
#include "hackflight.h"
#include "led.h"
#include "msp.h"

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

    friend class Task;

    public: // to support tasks

        arming_t     m_arming;
        Gyro         m_gyro;
        imu_fusion_t m_imuFusionPrev;

    private:

        // Scheduling constants
        static const uint32_t RX_TASK_RATE       = 33;
        static const uint32_t ATTITUDE_TASK_RATE = 100;

        // Essential tasks
        AttitudeTask  m_attitudeTask;
        //ReceiverTask  m_rxTask;

        imu_align_fun    m_imuAlignFun;
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

    public:

        Hackflight(
                rx_dev_funs_t * rxDeviceFuns,
                serialPortIdentifier_e rxDevPort,
                anglePidConstants_t * anglePidConstants,
                mixer_t mixer,
                void * motorDevice,
                uint8_t imuInterruptPin,
                imu_align_fun imuAlign,
                uint8_t ledPin) 
            : HackflightCore(anglePidConstants, mixer)
        {
            (void)imuAlign;

            mspInit();
            imuInit(imuInterruptPin);
            ledInit(ledPin);
            ledFlash(10, 50);
            failsafeInit();
            failsafeReset();

            m_rx.devCheck = rxDeviceFuns->check;
            m_rx.devConvert = rxDeviceFuns->convert;

            rxDeviceFuns->init(rxDevPort);

            m_imuAlignFun = imuAlign;

            m_motorDevice = motorDevice;

#if 0
            initTask(&m_rxTask, task_rx,  RX_TASK_RATE);

            // Initialize quaternion in upright position
            m_imuFusionPrev.quat.w = 1;

            m_maxArmingAngle = deg2rad(MAX_ARMING_ANGLE);

            initTask(&m_mspTask, task_msp, MSP_TASK_RATE);

            scheduler_t * scheduler = &m_scheduler;

            scheduler->loopStartCycles =
                systemClockMicrosToCycles(SCHED_START_LOOP_MIN_US);
            scheduler->loopStartMinCycles =
                systemClockMicrosToCycles(SCHED_START_LOOP_MIN_US);
            scheduler->loopStartMaxCycles =
                systemClockMicrosToCycles(SCHED_START_LOOP_MAX_US);
            scheduler->loopStartDeltaDownCycles =
                systemClockMicrosToCycles(1) / SCHED_START_LOOP_DOWN_STEP;
            scheduler->loopStartDeltaUpCycles =
                systemClockMicrosToCycles(1) / SCHED_START_LOOP_UP_STEP;

            scheduler->taskGuardMinCycles =
                systemClockMicrosToCycles(TASK_GUARD_MARGIN_MIN_US);
            scheduler->taskGuardMaxCycles =
                systemClockMicrosToCycles(TASK_GUARD_MARGIN_MAX_US);
            scheduler->taskGuardCycles = scheduler->taskGuardMinCycles;
            scheduler->taskGuardDeltaDownCycles =
                systemClockMicrosToCycles(1) / TASK_GUARD_MARGIN_DOWN_STEP;
            scheduler->taskGuardDeltaUpCycles =
                systemClockMicrosToCycles(1) / TASK_GUARD_MARGIN_UP_STEP;

            scheduler->lastTargetCycles = systemGetCycleCounter();

            scheduler->nextTimingCycles = scheduler->lastTargetCycles;

            scheduler->desiredPeriodCycles =
                (int32_t)systemClockMicrosToCycles(CORE_PERIOD());

            scheduler->guardMargin =
                (int32_t)systemClockMicrosToCycles(CHECK_GUARD_MARGIN_US);

            scheduler->clockRate = systemClockMicrosToCycles(1000000);        

#endif
        }

        void step(void)
        {
        }
};
