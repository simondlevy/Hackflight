/*
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

#include <stdint.h>

#include <vector>
using namespace std;

#include "arming.h"
#include "core/mixer.h"
#include "esc.h"
#include "imu.h"
#include "led.h"
#include "maths.h"
#include "scheduler.h"
#include "tasks/attitude.h"
#include "tasks/receiver.h"

class Board {

    private:

        // Gyro interrupt counts over which to measure loop time and skew
        static const uint32_t CORE_RATE_COUNT = 25000;
        static const uint32_t GYRO_LOCK_COUNT = 400;

        uint8_t m_ledPin;

        // Initialzed in main()
        Imu *                     m_imu;
        Esc *                     m_esc;
        Led *                     m_led;
        Mixer *                   m_mixer;
        vector<PidController *> * m_pidControllers;
        Receiver *                m_receiver;

        // Initialzed here
        Arming         m_arming;
        AttitudeTask   m_attitude;
        bool           m_failsafeIsActive;
        Imu::align_fun m_imuAlignFun;
        // Msp            m_msp;
        Scheduler      m_scheduler;
        VehicleState   m_vstate;

     protected:

        Board(const uint8_t ledPin)
        {
            m_ledPin = ledPin;
        }

    public:

        uint32_t microsToCycles(uint32_t micros)
        {
            return getClockSpeed() / 1000000 * micros;
        }

        virtual uint32_t getClockSpeed(void)  = 0;

        virtual uint32_t getCycleCounter(void) = 0;

        virtual void reboot(void) { }

        virtual void startCycleCounter(void) = 0;

        virtual void dmaInit(uint32_t outputFreq)
        {
            (void)outputFreq;
        }

        virtual void dmaUpdateComplete(void)
        {
        }

        virtual void dmaUpdateStart(void)
        {
        }

        virtual void dmaWriteMotor(uint8_t index, uint16_t packet)
        {
            (void)index;
            (void)packet;
        }
}; 
