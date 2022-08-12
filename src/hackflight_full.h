/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "arming.h"
#include "attitude_task.h"
#include "debug.h"
#include "deg2rad.h"
#include "failsafe.h"
#include "gyro.h"
#include "hackflight_core.h"
#include "led.h"
#include "motor.h"
#include "msp_task.h"
#include "msp.h"
#include "receiver_task.h"
#include "rx.h"
#include "scheduler.h"
#include "system.h"
#include "task.h"

// Gyro interrupt counts over which to measure loop time and skew
static const uint32_t CORE_RATE_COUNT = 25000;
static const uint32_t GYRO_LOCK_COUNT = 400;

// Arming safety  
static const float MAX_ARMING_ANGLE = 25;

// Full structure for running Hackflight

typedef struct {

    hackflight_core_t core;

    imu_align_fun imuAlignFun;
    task_data_t   taskData;
    Scheduler     scheduler;

    AttitudeTask attitudeTask;
    MspTask      mspTask;
    ReceiverTask receiverTask;

} hackflight_full_t;

static void checkCoreTasks(
        hackflight_full_t * full,
        Scheduler * scheduler,
        uint32_t nowCycles)
{
    hackflight_core_t * core = &full->core;
    task_data_t * td = &full->taskData;

    int32_t loopRemainingCycles = scheduler->getLoopRemainingCycles();
    uint32_t nextTargetCycles = scheduler->getNextTargetCycles();

    scheduler->corePreUpdate();

    while (loopRemainingCycles > 0) {
        nowCycles = systemGetCycleCounter();
        loopRemainingCycles = cmpTimeCycles(nextTargetCycles, nowCycles);
    }

    gyroReadScaled(&td->gyro, full->imuAlignFun, &core->vstate);

    uint32_t usec = timeMicros();

    rxGetDemands(&td->rx, usec, &core->anglePid, &core->demands);

    float mixmotors[MAX_SUPPORTED_MOTORS] = {0};

    motor_config_t motorConfig = {
        motorValueDisarmed(),
        motorValueHigh(),
        motorValueLow(),
        motorIsProtocolDshot()  
    };

    hackflightRunCoreTasks(
            core,
            usec,
            failsafeIsActive(),
            &motorConfig,
            mixmotors);

    motorWrite(td->motorDevice,
            armingIsArmed(&td->arming) ? mixmotors : td->mspMotors);

    scheduler->corePostUpdate(nowCycles);

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
}

static void checkDynamicTasks( hackflight_full_t * full, Scheduler * scheduler)
{
    hackflight_core_t * core = &full->core;
    task_data_t * td = &full->taskData;

    Task *selectedTask = NULL;
    uint16_t selectedTaskDynamicPriority = 0;

    uint32_t usec = timeMicros();

    Task::update(&full->receiverTask, &full->taskData, usec,
            &selectedTask, &selectedTaskDynamicPriority);

    Task::update(&full->attitudeTask, &full->taskData, usec,
            &selectedTask, &selectedTaskDynamicPriority);

    Task::update(&full->mspTask, &full->taskData, usec,
            &selectedTask, &selectedTaskDynamicPriority);

    if (selectedTask) {

        int32_t loopRemainingCycles = scheduler->getLoopRemainingCycles();
        uint32_t nextTargetCycles = scheduler->getNextTargetCycles();

        int32_t taskRequiredTimeUs = selectedTask->getRequiredTime();
        int32_t taskRequiredCycles =
            (int32_t)systemClockMicrosToCycles((uint32_t)taskRequiredTimeUs);

        uint32_t nowCycles = systemGetCycleCounter();
        loopRemainingCycles = cmpTimeCycles(nextTargetCycles, nowCycles);

        // Allow a little extra time
        taskRequiredCycles += scheduler->getTaskGuardCycles();

        if (taskRequiredCycles < loopRemainingCycles) {

            uint32_t anticipatedEndCycles = nowCycles + taskRequiredCycles;

            selectedTask->execute(core, td, usec);

            scheduler->updateDynamic(
                    systemGetCycleCounter(),
                    anticipatedEndCycles);

        } else {
            selectedTask->enableRun();
        }
    }
}

// -------------------------------------------------------------------------

void hackflightInitFull(
        hackflight_full_t * full,
        rx_dev_funs_t * rxDeviceFuns,
        serialPortIdentifier_e rxDevPort,
        anglePidConstants_t * anglePidConstants,
        mixer_t mixer,
        void * motorDevice,
        uint8_t imuInterruptPin,
        imu_align_fun imuAlign,
        uint8_t ledPin)
{
    hackflight_core_t * core = &full->core;

    hackflightInit(core, anglePidConstants, mixer);

    task_data_t * td = &full->taskData;

    mspInit();
    gyroInit(&td->gyro);
    imuInit(imuInterruptPin);
    ledInit(ledPin);
    ledFlash(10, 50);
    failsafeInit();
    failsafeReset();

    td->rx.devCheck = rxDeviceFuns->check;
    td->rx.devConvert = rxDeviceFuns->convert;

    rxDeviceFuns->init(rxDevPort);

    full->imuAlignFun = imuAlign;

    td->motorDevice = motorDevice;

    // Initialize quaternion in upright position
    td->imuFusionPrev.quat.w = 1;

    td->maxArmingAngle = deg2rad(MAX_ARMING_ANGLE);
}

void hackflightStep(hackflight_full_t * full)
{
    Scheduler * scheduler = &full->scheduler;

    // Realtime gyro/filtering/PID tasks get complete priority
    uint32_t nowCycles = systemGetCycleCounter();

    if (scheduler->isCoreReady(nowCycles)) {
        checkCoreTasks(full, scheduler, nowCycles);
    }

    if (scheduler->isDynamicReady(systemGetCycleCounter())) {
        checkDynamicTasks(full, scheduler);
    }
}

class Hackflight : public HackflightCore {

    private:

        imu_align_fun m_imuAlignFun;
        task_data_t   m_taskData;
        Scheduler     m_scheduler;

        AttitudeTask m_attitudeTask;
        MspTask      m_mspTask;
        ReceiverTask m_receiverTask;

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
            task_data_t * td = &m_taskData;

            mspInit();
            gyroInit(&td->gyro);
            imuInit(imuInterruptPin);
            ledInit(ledPin);
            ledFlash(10, 50);
            failsafeInit();
            failsafeReset();

            td->rx.devCheck = rxDeviceFuns->check;
            td->rx.devConvert = rxDeviceFuns->convert;

            rxDeviceFuns->init(rxDevPort);

            m_imuAlignFun = imuAlign;

            td->motorDevice = motorDevice;

            // Initialize quaternion in upright position
            td->imuFusionPrev.quat.w = 1;

            td->maxArmingAngle = deg2rad(MAX_ARMING_ANGLE);
        }

}; // class Hackflight
