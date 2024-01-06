/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * sysload.c - System load monitor
 */

#include <free_rtos.h>

#include <cfassert.h>
#include <console.h>
#include <sysload.h>
#include <timers.h>

#define TIMER_PERIOD M2T(1000)

// Shared with params
uint8_t sysload_triggerDump = 0;

static void timerHandler(xTimerHandle timer);

static bool initialized = false;


typedef struct {
  uint32_t ulRunTimeCounter;
  uint32_t xTaskNumber;
} taskData_t;

#define TASK_MAX_COUNT 32
static taskData_t previousSnapshot[TASK_MAX_COUNT];

static int taskTopIndex;

static uint32_t previousTotalRunTime;

static StaticTimer_t timerBuffer;



static taskData_t* getPreviousTaskData(uint32_t xTaskNumber) {
  // Try to find the task in the list of tasks
  for (int i = 0; i < taskTopIndex; i++) {
    if (previousSnapshot[i].xTaskNumber == xTaskNumber) {
      return &previousSnapshot[i];
    }
  }

  // Allocate a new entry
  taskData_t* result = &previousSnapshot[taskTopIndex];
  result->xTaskNumber = xTaskNumber;

  taskTopIndex++;

  return result;
}

static void timerHandler(xTimerHandle timer) {
  if (sysload_triggerDump != 0) {
    uint32_t totalRunTime;

    TaskStatus_t taskStats[TASK_MAX_COUNT];
    uint32_t taskCount = uxTaskGetSystemState(taskStats, TASK_MAX_COUNT, &totalRunTime);

    uint32_t totalDelta = totalRunTime - previousTotalRunTime;
    float f = 100.0 / totalDelta;

    // Dumps the the CPU load and stack usage for all tasks
    // CPU usage is since last dump in % compared to total time spent in tasks.
    // Note that time spent in interrupts will be included in measured time.
    // Stack usage is displayed as nr of unused bytes at peak stack usage.

    consolePrintf("SYSLOAD: Task dump\n");
    consolePrintf("SYSLOAD: Load\tStack left\tName\n");
    for (uint32_t i = 0; i < taskCount; i++) {
      TaskStatus_t* stats = &taskStats[i];
      taskData_t* previousTaskData = getPreviousTaskData(stats->xTaskNumber);

      uint32_t taskRunTime = stats->ulRunTimeCounter;
      float load = f * (taskRunTime - previousTaskData->ulRunTimeCounter);
      consolePrintf("SYSLOAD: %.2f \t%u \t%s\n", 
              (double)load, stats->usStackHighWaterMark, stats->pcTaskName);

      previousTaskData->ulRunTimeCounter = taskRunTime;
    }

    previousTotalRunTime = totalRunTime;

    sysload_triggerDump = 0;
  }
}

//////////////////////////////////////////////////////////////////////////////

void sysLoadInit() 
{
  xTimerHandle timer = xTimerCreateStatic( "sysLoadMonitorTimer", TIMER_PERIOD,
          pdTRUE, NULL, timerHandler, &timerBuffer);

  xTimerStart(timer, 100);

  initialized = true;
}
