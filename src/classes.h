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

class HackflightCore {

    private:

        // PID-limiting constants

        static constexpr float    PID_MIXER_SCALING = 1000;
        static constexpr uint16_t PIDSUM_LIMIT_YAW  = 400;
        static constexpr uint16_t PIDSUM_LIMIT      = 500;

        // Instance variable

        anglePid_t       m_anglepid;
        demands_t        m_demands;
        mixer_t          m_mixer;
        pid_controller_t m_pidControllers[10];
        uint8_t          m_pidCount;
        bool             m_pidReset;

        static float constrain_demand(float demand, float limit, float scaling)
        {
            return constrain_f(demand, -limit, +limit) / scaling;
        }

    public:

        // Supports tasks
        vehicle_state_t  m_vstate;

        HackflightCore(anglePidConstants_t * anglePidConstants, mixer_t mixer)
        {
            m_mixer = mixer;

            anglePidInit(&m_anglepid, anglePidConstants);

            addPidController(anglePidUpdate, &m_anglepid);
        }

        void addPidController(pid_fun_t fun, void * data)
        {
            m_pidControllers[m_pidCount].fun = fun;
            m_pidControllers[m_pidCount].data = data;
            m_pidCount++;
        }

        void runCoreTasks(
                uint32_t usec,
                bool failsafe,
                motor_config_t * motorConfig,
                float motorvals[])
        {
            // Run PID controllers to get new demands
            for (uint8_t k=0; k<m_pidCount; ++k) {
                pid_controller_t pid = m_pidControllers[k];
                pid.fun(usec, &m_demands, pid.data, &m_vstate, m_pidReset);
            }

            // Constrain the demands, negating yaw to make it agree with PID
            demands_t * demands = &m_demands;
            demands->roll  =
                constrain_demand(demands->roll, PIDSUM_LIMIT, PID_MIXER_SCALING);
            demands->pitch =
                constrain_demand(demands->pitch, PIDSUM_LIMIT, PID_MIXER_SCALING);
            demands->yaw   =
                -constrain_demand(demands->yaw, PIDSUM_LIMIT_YAW, PID_MIXER_SCALING);

            // Run the mixer to get motors from demands
            m_mixer(&m_demands, failsafe, motorConfig, motorvals);
        }

}; // class Hackflight

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

    // Gyro interrupt counts over which to measure loop time and skew
    static const uint32_t CORE_RATE_COUNT = 25000;
    static const uint32_t GYRO_LOCK_COUNT = 400;

    // Essential tasks
    AttitudeTask  m_attitudeTask;
    MspTask       m_mspTask;
    ReceiverTask  m_rxTask;

    // Demands
    demands_t m_demands;

    // Essential PID controller
    anglePid_t m_anglePid;

    // Scheduler
    scheduler_t m_scheduler;

    // IMU alignment function
    imu_align_fun m_imuAlignFun;

    // Core contents for tasks
    task_data_t m_taskData;

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

        task_data_t * data = &m_taskData;

        gyroReadScaled(&data->gyro, m_imuAlignFun, &data->vstate);

        uint32_t usec = timeMicros();

        rxGetDemands(&data->rx, usec, &m_anglePid, &m_demands);

        motor_config_t motorConfig = {
            motorValueDisarmed(),
            motorValueHigh(),
            motorValueLow(),
            motorIsProtocolDshot()  
        };

        float mixmotors[MAX_SUPPORTED_MOTORS] = {};

        runCoreTasks(
                usec,
                failsafeIsActive(),
                &motorConfig,
                mixmotors);

        motorWrite(
                data->motorDevice,
                armingIsArmed(&data->arming) ? mixmotors : data->mspMotors);

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
    }

    void adjustRxDynamicPriority(uint32_t usec) 
    {
        if (m_rxTask.dynamicPriority > 0) {
            m_rxTask.ageCycles = 1 +
                (cmpTimeUs(usec, m_rxTask.lastSignaledAtUs) /
                 m_rxTask.desiredPeriodUs);
            m_rxTask.dynamicPriority = 1 + m_rxTask.ageCycles;
        } else  {
            if (rxCheck(&m_taskData.rx, usec)) {
                m_rxTask.lastSignaledAtUs = usec;
                m_rxTask.ageCycles = 1;
                m_rxTask.dynamicPriority = 2;
            } else {
                m_rxTask.ageCycles = 0;
            }
        }
    }

    void updateDynamicTask(Task * task, Task ** selected,
            uint16_t * selectedPriority)
    {
        if (task->dynamicPriority > *selectedPriority) {
            *selectedPriority = task->dynamicPriority;
            *selected = task;
        }
    }

    void adjustAndUpdateTask(
            Task * task,
            uint32_t usec,
            Task ** selectedTask,
            uint16_t * selectedTaskDynamicPriority)
    {
        adjustDynamicPriority(task, usec);
        updateDynamicTask(task, selectedTask, selectedTaskDynamicPriority);
    }



    void checkDynamicTasks(
            int32_t loopRemainingCycles,
            uint32_t nextTargetCycles)
    {
        Task * selectedTask = NULL;
        uint16_t selectedTaskDynamicPriority = 0;

        uint32_t usec = timeMicros();

        adjustRxDynamicPriority(usec);

        updateDynamicTask(&m_rxTask, &selectedTask,
                &selectedTaskDynamicPriority);

        adjustAndUpdateTask(&m_attitudeTask, usec,
                &selectedTask, &selectedTaskDynamicPriority);

        adjustAndUpdateTask(&m_mspTask, usec,
                &selectedTask, &selectedTaskDynamicPriority);

        if (selectedTask) {

            int32_t taskRequiredTimeUs =
                selectedTask->anticipatedExecutionTime >> TASK_EXEC_TIME_SHIFT;
            int32_t taskRequiredTimeCycles =
                (int32_t)systemClockMicrosToCycles((uint32_t)taskRequiredTimeUs);

            uint32_t nowCycles = systemGetCycleCounter();
            loopRemainingCycles = cmpTimeCycles(nextTargetCycles, nowCycles);

            scheduler_t * scheduler = &m_scheduler;

            // Allow a little extra time
            taskRequiredTimeCycles += scheduler->taskGuardCycles;

            if (taskRequiredTimeCycles < loopRemainingCycles) {
                uint32_t antipatedEndCycles =
                    nowCycles + taskRequiredTimeCycles;
                executeTask(selectedTask, usec);
                nowCycles = systemGetCycleCounter();
                int32_t cyclesOverdue =
                    cmpTimeCycles(nowCycles, antipatedEndCycles);

                if ((cyclesOverdue > 0) ||
                        (-cyclesOverdue < scheduler->taskGuardMinCycles)) {
                    if (scheduler->taskGuardCycles <
                            scheduler->taskGuardMaxCycles) {
                        scheduler->taskGuardCycles +=
                            scheduler->taskGuardDeltaUpCycles;
                    }
                } else if (scheduler->taskGuardCycles >
                        scheduler->taskGuardMinCycles) {
                    scheduler->taskGuardCycles -=
                        scheduler->taskGuardDeltaDownCycles;
                }
            } else if (selectedTask->ageCycles > TASK_AGE_EXPEDITE_COUNT) {
                // If a task has been unable to run, then reduce it's recorded
                // estimated run time to ensure it's ultimate scheduling
                selectedTask->anticipatedExecutionTime *= 
                    TASK_AGE_EXPEDITE_SCALE;
            }
        }
    }

    void executeTask(Task *task, uint32_t usec)
    {
        task->lastExecutedAtUs = usec;
        task->dynamicPriority = 0;

        uint32_t time = timeMicros();
        task->fun(&m_taskData, usec);

        uint32_t taskExecutionTimeUs = timeMicros() - time;

        // Update estimate of expected task duration
        if (taskExecutionTimeUs >
                (task->anticipatedExecutionTime >> TASK_EXEC_TIME_SHIFT)) {
            task->anticipatedExecutionTime =
                taskExecutionTimeUs << TASK_EXEC_TIME_SHIFT;
        } else if (task->anticipatedExecutionTime > 1) {
            // Slowly decay the max time
            task->anticipatedExecutionTime--;
        }
    }


    static void adjustDynamicPriority(Task * task, uint32_t usec) 
    {
        // Task is time-driven, dynamicPriority is last execution age
        // (measured in desiredPeriods). Task age is calculated from last
        // execution.
        task->ageCycles = (cmpTimeUs(usec, task->lastExecutedAtUs) /
                task->desiredPeriodUs);
        if (task->ageCycles > 0) {
            task->dynamicPriority = 1 + task->ageCycles;
        }
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
        gyroInit(&m_taskData.gyro);
        imuInit(imuInterruptPin);
        ledInit(ledPin);
        ledFlash(10, 50);
        failsafeInit();
        failsafeReset();

        m_taskData.rx.devCheck = rxDeviceFuns->check;
        m_taskData.rx.devConvert = rxDeviceFuns->convert;

        rxDeviceFuns->init(rxDevPort);

        m_imuAlignFun = imuAlign;

        m_taskData.motorDevice = motorDevice;

        // Initialize quaternion in upright position
        m_taskData.imuFusionPrev.quat.w = 1;

        m_taskData.maxArmingAngle = deg2rad(MAX_ARMING_ANGLE);

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

        // Once close to the timing boundary, poll for its arrival
        if (loopRemainingCycles < m_scheduler.loopStartCycles) {
            checkCoreTasks(
                    loopRemainingCycles,
                    nowCycles,
                    nextTargetCycles);
        }

        int32_t newLoopRemainingCyles =
            cmpTimeCycles(nextTargetCycles, systemGetCycleCounter());

        if (newLoopRemainingCyles > m_scheduler.guardMargin) {
            checkDynamicTasks(newLoopRemainingCyles, nextTargetCycles);
        }

    } // step()
};
