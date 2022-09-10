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

#include <stdbool.h>
#include <stdint.h>

class Task {

    private:

        // Some tasks have occasional peaks in execution time so normal moving
        // average duration estimation doesn't work Decay the estimated max
        // task duration by 1/(1 << EXEC_TIME_SHIFT) on every invocation
        static const uint32_t EXEC_TIME_SHIFT = 7;

        // Make aged tasks more schedulable
        static const uint32_t AGE_EXPEDITE_COUNT = 1;   

        // By scaling their expected execution time
        static constexpr float AGE_EXPEDITE_SCALE = 0.9; 

        uint32_t m_anticipatedExecutionTime;

    protected:

        uint16_t m_ageCycles;
        int32_t  m_desiredPeriodUs;            
        uint16_t m_dynamicPriority;          
        uint32_t m_lastExecutedAtUs;          
        uint32_t m_lastSignaledAtUs;         

        Task(uint32_t rate) 
        {
            m_desiredPeriodUs = 1000000 / rate;
        }

    public:

        virtual void adjustDynamicPriority(uint32_t usec)
        {
            // Task is time-driven, dynamicPriority is last execution age
            // (measured in desiredPeriods). Task age is calculated from last
            // execution.
            m_ageCycles =
                (cmpTimeUs(usec, m_lastExecutedAtUs) / m_desiredPeriodUs);
            if (m_ageCycles > 0) {
                m_dynamicPriority = 1 + m_ageCycles;
            }
        }

        // If a task has been unable to run, then reduce its recorded
        // estimated run time to ensure its ultimate scheduling
        void enableRun(void) 
        {
            if (m_ageCycles > AGE_EXPEDITE_COUNT) {
                m_anticipatedExecutionTime *= AGE_EXPEDITE_SCALE;
            }
        }

        void execute(uint32_t usec)
        {
            m_lastExecutedAtUs = usec;
            m_dynamicPriority = 0;

            uint32_t time = timeMicros();
            fun(usec);

            uint32_t taskExecutionTimeUs = timeMicros() - time;

            if (taskExecutionTimeUs >
                    (m_anticipatedExecutionTime >> EXEC_TIME_SHIFT)) {
                m_anticipatedExecutionTime =
                    taskExecutionTimeUs << EXEC_TIME_SHIFT;
            } else if (m_anticipatedExecutionTime > 1) {
                // Slowly decay the max time
                m_anticipatedExecutionTime--;
            }
        }

        int32_t getRequiredTime(void)
        {
            return m_anticipatedExecutionTime >> EXEC_TIME_SHIFT;
        }

        void update(uint32_t usec, Task ** selected, uint16_t * selectedPriority)
        {
            adjustDynamicPriority(usec);

            if (m_dynamicPriority > *selectedPriority) {
                *selectedPriority = m_dynamicPriority;
                *selected = this;
            }
        }

        virtual void fun(uint32_t usec) = 0;
}; 
