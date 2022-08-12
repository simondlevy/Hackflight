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

class Scheduler {

    private:

        // Constants -------------------------------------------------------

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

        // Add a margin to the amount of time allowed for a check function to run
        static const uint32_t CHECK_GUARD_MARGIN_US = 2 ;  

        int32_t m_loopStartCycles;
        int32_t m_loopStartMinCycles;
        int32_t m_loopStartMaxCycles;
        uint32_t m_loopStartDeltaDownCycles;
        uint32_t m_loopStartDeltaUpCycles;

        uint32_t m_nextTimingCycles;

        int32_t m_guardMargin;
        uint32_t m_clockRate;

    public:

        int32_t desiredPeriodCycles;
        uint32_t lastTargetCycles;
        int32_t taskGuardCycles;
        int32_t taskGuardMinCycles;
        int32_t taskGuardMaxCycles;
        uint32_t taskGuardDeltaDownCycles;
        uint32_t taskGuardDeltaUpCycles;
        uint32_t nextTargetCycles;
        int32_t loopRemainingCycles;
        int32_t newLoopRemainingCyles;


        Scheduler(void)
        {
            m_loopStartCycles =
                systemClockMicrosToCycles(SCHED_START_LOOP_MIN_US);
            m_loopStartMinCycles =
                systemClockMicrosToCycles(SCHED_START_LOOP_MIN_US);
            m_loopStartMaxCycles =
                systemClockMicrosToCycles(SCHED_START_LOOP_MAX_US);
            m_loopStartDeltaDownCycles =
                systemClockMicrosToCycles(1) / SCHED_START_LOOP_DOWN_STEP;
            m_loopStartDeltaUpCycles =
                systemClockMicrosToCycles(1) / SCHED_START_LOOP_UP_STEP;

            taskGuardMinCycles =
                systemClockMicrosToCycles(TASK_GUARD_MARGIN_MIN_US);
            taskGuardMaxCycles =
                systemClockMicrosToCycles(TASK_GUARD_MARGIN_MAX_US);
            taskGuardCycles = taskGuardMinCycles;
            taskGuardDeltaDownCycles =
                systemClockMicrosToCycles(1) / TASK_GUARD_MARGIN_DOWN_STEP;
            taskGuardDeltaUpCycles =
                systemClockMicrosToCycles(1) / TASK_GUARD_MARGIN_UP_STEP;

            lastTargetCycles = systemGetCycleCounter();

            m_nextTimingCycles = lastTargetCycles;

            desiredPeriodCycles =
                (int32_t)systemClockMicrosToCycles(CORE_PERIOD());

            m_guardMargin =
                (int32_t)systemClockMicrosToCycles(CHECK_GUARD_MARGIN_US);

            m_clockRate = systemClockMicrosToCycles(1000000);
        }

        void corePreUpdate(void) 
        {
            if (m_loopStartCycles > m_loopStartMinCycles) {
                m_loopStartCycles -= m_loopStartDeltaDownCycles;
            }
        }

        void corePostUpdate(uint32_t nowCycles)
        {
            // CPU busy
            if (cmpTimeCycles(m_nextTimingCycles, nowCycles) < 0) {
                m_nextTimingCycles += m_clockRate;
            }
            lastTargetCycles = nextTargetCycles;
        }

        bool isCoreReady(uint32_t nowCycles)
        {
            nextTargetCycles = lastTargetCycles + desiredPeriodCycles;

            loopRemainingCycles = cmpTimeCycles(nextTargetCycles, nowCycles);

            if (loopRemainingCycles < -desiredPeriodCycles) {
                // A task has so grossly overrun that at entire gyro cycle has
                // been skipped This is most likely to occur when connected to
                // the configurator via USB as the serial task is
                // non-deterministic Recover as best we can, advancing
                // scheduling by a whole number of cycles
                nextTargetCycles += desiredPeriodCycles * (1 +
                        (loopRemainingCycles / -desiredPeriodCycles));
                loopRemainingCycles = cmpTimeCycles(nextTargetCycles, nowCycles);
            }

            // Tune out the time lost between completing the last task
            // execution and re-entering the scheduler
            if ((loopRemainingCycles < m_loopStartMinCycles) &&
                    (m_loopStartCycles < m_loopStartMaxCycles)) {
                m_loopStartCycles += m_loopStartDeltaUpCycles;
            }

            // Once close to the timing boundary, poll for its arrival
            return loopRemainingCycles < m_loopStartCycles;
        }

        bool isDynamicReady(uint32_t nowCycles) 
        {
            newLoopRemainingCyles = cmpTimeCycles(nextTargetCycles, nowCycles);

            return newLoopRemainingCyles > m_guardMargin;
        }

}; // class Scheduler
