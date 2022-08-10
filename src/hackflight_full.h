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
#include "deg2rad.h"
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

    private:

        // Wait at start of scheduler loop if gyroSampleTask is nearly due
        static const uint32_t SCHED_START_LOOP_MIN_US = 1;   
        static const uint32_t SCHED_START_LOOP_MAX_US = 12;

        // Fraction of a us to reduce start loop wait
        static const uint32_t SCHED_START_LOOP_DOWN_STEP = 50;  

        // Fraction of a us to increase start loop wait
        static const uint32_t SCHED_START_LOOP_UP_STEP = 1;   

        // Add an amount to the estimate of a task duration
        static const uint32_t TASK_GUARD_MARGIN_MIN_US = 3;   
        static const uint32_t TASK_GUARD_MARGIN_MAX_US = 6;

        // Fraction of a us to reduce task guard margin
        static const uint32_t TASK_GUARD_MARGIN_DOWN_STEP = 50;  

        // Fraction of a us to increase task guard margin
        static const uint32_t TASK_GUARD_MARGIN_UP_STEP = 1;   

        // Add a margin to the amount of time allowed for a check function to
        // run
        static const uint32_t CHECK_GUARD_MARGIN_US = 2 ;  

        // Some tasks have occasional peaks in execution time so normal moving
        // average duration estimation doesn't work Decay the estimated max
        // task duration by
        // 1/(1 << TASK_EXEC_TIME_SHIFT) on every invocation
        static const uint32_t TASK_EXEC_TIME_SHIFT = 7;

        // Make aged tasks more schedulable
        static const uint32_t TASK_AGE_EXPEDITE_COUNT = 1;   

        // By scaling their expected execution time
        static constexpr float TASK_AGE_EXPEDITE_SCALE = 0.9; 

        // Arming safety angle constant
        static constexpr float MAX_ARMING_ANGLE = 25;

        // Essential tasks
        AttitudeTask  m_attitudeTask;
        task_t        m_mspTask;
        ReceiverTask  m_rxTask;

        new_hackflight_t m_hackflight;

        imu_align_fun    m_imuAlignFun;
        float            m_maxArmingAngle;
        void *           m_motorDevice;
        float            m_mspMotors[4];
        rx_t             m_rx;
        rx_axes_t        m_rxAxes;
        scheduler_t      m_scheduler;
        task_t           m_sensorTasks[10];
        uint8_t          m_sensorTaskCount;

        void checkCoreTasks(
                int32_t loopRemainingCycles,
                uint32_t nowCycles,
                uint32_t nextTargetCycles)
        {
            scheduler_t * scheduler = &m_scheduler;

            if (scheduler->loopStartCycles > scheduler->loopStartMinCycles) {
                scheduler->loopStartCycles -= scheduler->loopStartDeltaDownCycles;
            }

            while (loopRemainingCycles > 0) {
                nowCycles = systemGetCycleCounter();
                loopRemainingCycles = cmpTimeCycles(nextTargetCycles, nowCycles);
            }

            /*
            gyroReadScaled(&hf->gyro, hf->imuAlignFun, &hf->vstate);

            uint32_t usec = timeMicros();

            rxGetDemands(&hf->rx, usec, &hf->anglepid, &hf->demands);

            float mixmotors[MAX_SUPPORTED_MOTORS] = {0};

            motor_config_t motorConfig = {
                motorValueDisarmed(),
                motorValueHigh(),
                motorValueLow(),
                motorIsProtocolDshot()  
            };

            hackflightRunCoreTasks(
                    hf,
                    usec,
                    failsafeIsActive(),
                    &motorConfig,
                    mixmotors);

            motorWrite(hf->motorDevice,
                    armingIsArmed(&hf->arming) ? mixmotors : hf->mspMotors);

            // CPU busy
            if (cmpTimeCycles(scheduler->nextTimingCycles, nowCycles) < 0) {
                scheduler->nextTimingCycles += scheduler->clockRate;
            }
            scheduler->lastTargetCycles = nextTargetCycles;

            // Bring the scheduler into lock with the gyro Track the actual gyro
            // rate over given number of cycle times and set the expected timebase
            static uint32_t _terminalGyroRateCount;
            static int32_t _sampleRateStartCycles;

            if ((_terminalGyroRateCount == 0)) {
                _terminalGyroRateCount = gyroInterruptCount() + CORE_RATE_COUNT;
                _sampleRateStartCycles = nowCycles;
            }

            if (gyroInterruptCount() >= _terminalGyroRateCount) {
                // Calculate number of clock cycles on average between gyro
                // interrupts
                uint32_t sampleCycles = nowCycles - _sampleRateStartCycles;
                scheduler->desiredPeriodCycles = sampleCycles / CORE_RATE_COUNT;
                _sampleRateStartCycles = nowCycles;
                _terminalGyroRateCount += CORE_RATE_COUNT;
            }

            // Track actual gyro rate over given number of cycle times and remove
            // skew
            static uint32_t _terminalGyroLockCount;
            static int32_t _gyroSkewAccum;

            int32_t gyroSkew =
                gyroGetSkew(nextTargetCycles, scheduler->desiredPeriodCycles);

            _gyroSkewAccum += gyroSkew;

            if ((_terminalGyroLockCount == 0)) {
                _terminalGyroLockCount = gyroInterruptCount() + GYRO_LOCK_COUNT;
            }

            if (gyroInterruptCount() >= _terminalGyroLockCount) {
                _terminalGyroLockCount += GYRO_LOCK_COUNT;

                // Move the desired start time of the gyroSampleTask
                scheduler->lastTargetCycles -= (_gyroSkewAccum/GYRO_LOCK_COUNT);

                _gyroSkewAccum = 0;
            }
            */
        }

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

            // Initialize quaternion in upright position
            m_imuFusionPrev.quat.w = 1;

            m_maxArmingAngle = deg2rad(MAX_ARMING_ANGLE);

            m_scheduler.loopStartCycles =
                systemClockMicrosToCycles(SCHED_START_LOOP_MIN_US);
            m_scheduler.loopStartMinCycles =
                systemClockMicrosToCycles(SCHED_START_LOOP_MIN_US);
            m_scheduler.loopStartMaxCycles =
                systemClockMicrosToCycles(SCHED_START_LOOP_MAX_US);
            m_scheduler.loopStartDeltaDownCycles =
                systemClockMicrosToCycles(1) / SCHED_START_LOOP_DOWN_STEP;
            m_scheduler.loopStartDeltaUpCycles =
                systemClockMicrosToCycles(1) / SCHED_START_LOOP_UP_STEP;

            m_scheduler.taskGuardMinCycles =
                systemClockMicrosToCycles(TASK_GUARD_MARGIN_MIN_US);
            m_scheduler.taskGuardMaxCycles =
                systemClockMicrosToCycles(TASK_GUARD_MARGIN_MAX_US);
            m_scheduler.taskGuardCycles = m_scheduler.taskGuardMinCycles;
            m_scheduler.taskGuardDeltaDownCycles =
                systemClockMicrosToCycles(1) / TASK_GUARD_MARGIN_DOWN_STEP;
            m_scheduler.taskGuardDeltaUpCycles =
                systemClockMicrosToCycles(1) / TASK_GUARD_MARGIN_UP_STEP;

            m_scheduler.lastTargetCycles = systemGetCycleCounter();

            m_scheduler.nextTimingCycles = m_scheduler.lastTargetCycles;

            m_scheduler.desiredPeriodCycles =
                (int32_t)systemClockMicrosToCycles(CORE_PERIOD());

            m_scheduler.guardMargin =
                (int32_t)systemClockMicrosToCycles(CHECK_GUARD_MARGIN_US);

            m_scheduler.clockRate = systemClockMicrosToCycles(1000000);        

        } // constructor

        void step(void)
        {
            uint32_t nextTargetCycles =
                m_scheduler.lastTargetCycles + m_scheduler.desiredPeriodCycles;

            // Realtime gyro/filtering/PID tasks get complete priority
            uint32_t nowCycles = systemGetCycleCounter();

            int32_t loopRemainingCycles =
                cmpTimeCycles(nextTargetCycles, nowCycles);

            if (loopRemainingCycles < -m_scheduler.desiredPeriodCycles) {
                // A task has so grossly overrun that at entire gyro cycle has
                // been skipped This is most likely to occur when connected to
                // the configurator via USB as the serial task is
                // non-deterministic Recover as best we can, advancing
                // scheduling by a whole number of cycles
                nextTargetCycles += m_scheduler.desiredPeriodCycles * (1 +
                        (loopRemainingCycles / -m_scheduler.desiredPeriodCycles));
                loopRemainingCycles = cmpTimeCycles(nextTargetCycles, nowCycles);
            }

            // Tune out the time lost between completing the last task
            // execution and re-entering the scheduler
            if ((loopRemainingCycles < m_scheduler.loopStartMinCycles) &&
                    (m_scheduler.loopStartCycles <
                     m_scheduler.loopStartMaxCycles)) {
                m_scheduler.loopStartCycles += m_scheduler.loopStartDeltaUpCycles;
            }

            /*
            // Once close to the timing boundary, poll for its arrival
            if (loopRemainingCycles < m_scheduler.loopStartCycles) {
                checkCoreTasks(
                        hf,
                        loopRemainingCycles,
                        nowCycles,
                        nextTargetCycles);
            }

            int32_t newLoopRemainingCyles =
                cmpTimeCycles(nextTargetCycles, systemGetCycleCounter());

            if (newLoopRemainingCyles > m_scheduler.guardMargin) {
                checkDynamicTasks(hf, newLoopRemainingCyles, nextTargetCycles);
            }
            */

        } // step()
};
