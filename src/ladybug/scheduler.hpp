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

#pragma once

#include <math.h>
#include <stdint.h>
#include <string.h>

#include <ladybug/pid.hpp>
#include <ladybug/task.hpp>

class Scheduler {

    private:

        // Constants -------------------------------------------------------

        // Wait at start of scheduler loop if gyroSampleTask is nearly due
        static const uint32_t START_LOOP_MIN_US = 1;   
        static const uint32_t START_LOOP_MAX_US = 12;

        // Fraction of a us to reduce start loop wait
        static const uint32_t START_LOOP_DOWN_STEP = 50;  

        // Fraction of a us to increase start loop wait
        static const uint32_t START_LOOP_UP_STEP = 1;   

        // Add an amount to the estimate of a task duration
        static const uint32_t TASK_GUARD_MARGIN_MIN_US = 3;   
        static const uint32_t TASK_GUARD_MARGIN_MAX_US = 6;

        // Fraction of a us to reduce task guard margin
        static const uint32_t TASK_GUARD_MARGIN_DOWN_STEP = 50;  

        // Fraction of a us to increase task guard margin
        static const uint32_t TASK_GUARD_MARGIN_UP_STEP = 1;   

        // Add a margin to the amount of time allowed for a check function to run
        static const uint32_t CHECK_GUARD_MARGIN_US = 2 ;  

        // State variables
        uint32_t m_clockRate;
        int32_t  m_guardMargin;
        int32_t  m_loopRemainingCycles;
        int32_t  m_loopStartCycles;
        uint32_t m_loopStartDeltaDownCycles;
        uint32_t m_loopStartDeltaUpCycles;
        int32_t  m_loopStartMaxCycles;
        int32_t  m_loopStartMinCycles;
        uint32_t m_nextTargetCycles;
        uint32_t m_nextTimingCycles;
        int32_t  m_taskGuardCycles;
        uint32_t m_taskGuardDeltaDownCycles;
        uint32_t m_taskGuardDeltaUpCycles;
        int32_t  m_taskGuardMinCycles;
        int32_t  m_taskGuardMaxCycles;

    public:

        // These can be modified by Board
        int32_t  desiredPeriodCycles;
        uint32_t lastTargetCycles;

        Scheduler(void)
        {
            m_loopStartCycles =
                microsecondsToClockCycles(START_LOOP_MIN_US);
            m_loopStartMinCycles =
                microsecondsToClockCycles(START_LOOP_MIN_US);
            m_loopStartMaxCycles =
                microsecondsToClockCycles(START_LOOP_MAX_US);
            m_loopStartDeltaDownCycles =
                microsecondsToClockCycles(1) / START_LOOP_DOWN_STEP;
            m_loopStartDeltaUpCycles =
                microsecondsToClockCycles(1) / START_LOOP_UP_STEP;

            m_taskGuardMinCycles =
                microsecondsToClockCycles(TASK_GUARD_MARGIN_MIN_US);
            m_taskGuardMaxCycles =
                microsecondsToClockCycles(TASK_GUARD_MARGIN_MAX_US);
            m_taskGuardCycles = m_taskGuardMinCycles;
            m_taskGuardDeltaDownCycles =
                microsecondsToClockCycles(1) / TASK_GUARD_MARGIN_DOWN_STEP;
            m_taskGuardDeltaUpCycles =
                microsecondsToClockCycles(1) / TASK_GUARD_MARGIN_UP_STEP;

            lastTargetCycles = 0;
            m_nextTimingCycles = 0;

            desiredPeriodCycles =
                (int32_t)microsecondsToClockCycles(PidController::PERIOD);

            m_guardMargin =
                (int32_t)microsecondsToClockCycles(CHECK_GUARD_MARGIN_US);

            m_clockRate = microsecondsToClockCycles(1000000);
        }

        uint32_t corePreUpdate(int32_t & loopRemainingCycles) 
        {
            if (m_loopStartCycles > m_loopStartMinCycles) {
                m_loopStartCycles -= m_loopStartDeltaDownCycles;
            }

            loopRemainingCycles = m_loopRemainingCycles;

            return m_nextTargetCycles;
        }

        void corePostUpdate(uint32_t nowCycles)
        {
            // CPU busy
            if (intcmp(m_nextTimingCycles, nowCycles) < 0) {
                m_nextTimingCycles += m_clockRate;
            }
            lastTargetCycles = m_nextTargetCycles;
        }

        uint32_t getAnticipatedEndCycles(LadybugTask & task, uint32_t nowCycles)
        {
            const uint32_t taskRequiredCycles = 
                task.checkReady(m_nextTargetCycles, nowCycles, m_taskGuardCycles);

            return taskRequiredCycles > 0 ? 
                    nowCycles + taskRequiredCycles :
                    0;
        }

        int32_t getTaskGuardCycles(void)
        {
            return m_taskGuardCycles;
        }
        
        bool isCoreReady(uint32_t nowCycles)
        {
            m_nextTargetCycles = lastTargetCycles + desiredPeriodCycles;

            m_loopRemainingCycles = intcmp(m_nextTargetCycles, nowCycles);

            if (m_loopRemainingCycles < -desiredPeriodCycles) {
                // A task has so grossly overrun that at entire gyro cycle has
                // been skipped This is most likely to occur when connected to
                // the configurator via USB as the serial task is
                // non-deterministic Recover as best we can, advancing
                // scheduling by a whole number of cycles
                m_nextTargetCycles += desiredPeriodCycles * (1 +
                        (m_loopRemainingCycles / -desiredPeriodCycles));
                m_loopRemainingCycles = intcmp(
                        m_nextTargetCycles, nowCycles);
            }

            // Tune out the time lost between completing the last task
            // execution and re-entering the scheduler
            if ((m_loopRemainingCycles < m_loopStartMinCycles) &&
                    (m_loopStartCycles < m_loopStartMaxCycles)) {
                m_loopStartCycles += m_loopStartDeltaUpCycles;
            }

            // Once close to the timing boundary, poll for its arrival
            return m_loopRemainingCycles < m_loopStartCycles;
        }

        bool isDynamicReady(uint32_t nowCycles) 
        {
            auto newLoopRemainingCyles =
                intcmp(m_nextTargetCycles, nowCycles);

            return newLoopRemainingCyles > m_guardMargin;
        }

        void updateDynamic(uint32_t nowCycles, uint32_t anticipatedEndCycles)
        {
            auto cyclesOverdue = intcmp(nowCycles, anticipatedEndCycles);

            if ((cyclesOverdue > 0) || (-cyclesOverdue < m_taskGuardMinCycles)) {

                if (m_taskGuardCycles < m_taskGuardMaxCycles) {
                    m_taskGuardCycles += m_taskGuardDeltaUpCycles;
                }
            } else if (m_taskGuardCycles > m_taskGuardMinCycles) {
                m_taskGuardCycles -= m_taskGuardDeltaDownCycles;
            }        
        }

}; // class Scheduler
