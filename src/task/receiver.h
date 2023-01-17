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

#include "receiver.h"
#include "task.h"

class ReceiverTask : public Task {

    friend class Board;

    private:

        // Set in Board constructor
        Receiver * receiver;

    protected:

        ReceiverTask()
            : Task(RECEIVER, 33) // Hz
        {
        }

        void run(const uint32_t usec)
        {
            receiver->update(usec);
        }

        // Increase priority for RX task
        virtual void adjustDynamicPriority(uint32_t usec) override
        {
            if (m_dynamicPriority > 0) {
                m_ageCycles = 1 + (intcmp(usec, m_lastSignaledAtUs) / m_desiredPeriodUs);
                m_dynamicPriority = 1 + m_ageCycles;
            } else  {
                if (receiver->check(usec)) {
                    m_lastSignaledAtUs = usec;
                    m_ageCycles = 1;
                    m_dynamicPriority = 2;
                } else {
                    m_ageCycles = 0;
                }
            }
        }    

}; // class ReceiverTask
