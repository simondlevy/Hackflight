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
#include <stdarg.h>

#include <vector>
using namespace std;

#include "core/mixer.h"
#include "core/motors.h"
#include "esc.h"
#include "imu.h"
#include "receiver.h"
#include "safety.h"
#include "scheduler.h"
#include "task/accelerometer.h"
#include "task/attitude.h"
#include "task/visualizer.h"
#include "task/receiver.h"

class Core {

    public:

        Scheduler scheduler;

        void completeTask(
                Imu * imu,
                const uint32_t imuInterruptCount,
                const uint32_t nowCycles,
                const uint32_t nextTargetCycles)
        {
            scheduler.corePostUpdate(nowCycles);

            // Bring the scheduler into lock with the gyro Track the actual
            // gyro rate over given number of cycle times and set the expected
            // timebase
            static uint32_t _terminalGyroRateCount;
            static int32_t _sampleRateStartCycles;

            if ((_terminalGyroRateCount == 0)) {
                _terminalGyroRateCount = imuInterruptCount + Imu::CORE_RATE_COUNT;
                _sampleRateStartCycles = nowCycles;
            }

            if (imuInterruptCount >= _terminalGyroRateCount) {
                // Calculate number of clock cycles on average between gyro
                // interrupts
                uint32_t sampleCycles = nowCycles - _sampleRateStartCycles;
                scheduler.desiredPeriodCycles = sampleCycles / Imu::CORE_RATE_COUNT;
                _sampleRateStartCycles = nowCycles;
                _terminalGyroRateCount += Imu::CORE_RATE_COUNT;
            }

            // Track actual gyro rate over given number of cycle times and
            // remove skew
            static uint32_t _terminalGyroLockCount;
            static int32_t _gyroSkewAccum;

            auto gyroSkew =
                imu->getGyroSkew(nextTargetCycles, scheduler.desiredPeriodCycles);

            _gyroSkewAccum += gyroSkew;

            if ((_terminalGyroLockCount == 0)) {
                _terminalGyroLockCount = imuInterruptCount + Imu::GYRO_LOCK_COUNT;
            }

            if (imuInterruptCount >= _terminalGyroLockCount) {
                _terminalGyroLockCount += Imu::GYRO_LOCK_COUNT;

                // Move the desired start time of the gyroSampleTask
                scheduler.lastTargetCycles -= (_gyroSkewAccum/Imu::GYRO_LOCK_COUNT);

                _gyroSkewAccum = 0;
            }
        }

}; // class Core
