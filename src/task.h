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

    public:

        typedef enum {
            NONE,
            ACCELEROMETER,
            ATTITUDE,
            VISUALIZER,
            RECEIVER,
            SKYRANGER
        } id_t;

        typedef struct {
            id_t id;
            uint16_t priority;
        } prioritizer_t;

    private:

        // Some task have occasional peaks in execution time so normal moving
        // average duration estimation doesn't work Decay the estimated max
        // task duration by 1/(1 << EXEC_TIME_SHIFT) on every invocation
        static const uint32_t EXEC_TIME_SHIFT = 7;

        // Make aged task more schedulable
        static const uint32_t AGE_EXPEDITE_COUNT = 1;   

        // By scaling their expected execution time
        static constexpr float AGE_EXPEDITE_SCALE = 0.9; 

        id_t m_id;

        uint32_t m_anticipatedExecutionTime;

    protected:

        uint16_t m_ageCycles;
        int32_t  m_desiredPeriodUs;            
        uint16_t m_dynamicPriority;          
        uint32_t m_lastExecutedAtUs;          
        uint32_t m_lastSignaledAtUs;         

        Task(const id_t id, const uint32_t rate) 
        {
            m_id = id;
            m_desiredPeriodUs = 1000000 / rate;
        }

    public:

        virtual void adjustDynamicPriority(const uint32_t usec)
        {
            // Task is time-driven, dynamicPriority is last execution age
            // (measured in desiredPeriods). Task age is calculated from last
            // execution.
            m_ageCycles =
                (intcmp(usec, m_lastExecutedAtUs) / m_desiredPeriodUs);
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

        void execute(const uint32_t usec)
        {
            fun(usec);

            m_lastExecutedAtUs = usec;
        }

        void update(const uint32_t executionTimeUs)
        {
            m_dynamicPriority = 0;

            if (executionTimeUs > (m_anticipatedExecutionTime >> EXEC_TIME_SHIFT)) {
                m_anticipatedExecutionTime = executionTimeUs << EXEC_TIME_SHIFT;
            } else if (m_anticipatedExecutionTime > 1) {
                // Slowly decay the max time
                m_anticipatedExecutionTime--;
            }
        }

        int32_t getRequiredTime(void)
        {
            return m_anticipatedExecutionTime >> EXEC_TIME_SHIFT;
        }

        virtual void prioritize(const uint32_t usec, prioritizer_t & prioritizer)
        {
            adjustDynamicPriority(usec);

            if (m_dynamicPriority > prioritizer.priority) {
                prioritizer.id = m_id;
                prioritizer.priority = m_dynamicPriority;
            }
        }

        virtual void fun(const uint32_t usec) = 0;
}; 
