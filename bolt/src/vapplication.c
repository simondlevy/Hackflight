/**
 *
 * Copyright (C) 2011-2022 Bitcraze AB, 2025 Simon D. Levy
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
 */

#include <free_rtos/FreeRTOS.h>
#include <free_rtos/semphr.h>
#include <free_rtos/task.h>

#include <motors_api.h>

static void fail(const char * msg)
{
    portDISABLE_INTERRUPTS();
    while (true);
}

void vApplicationIdleHook( void )
{
    // Enter sleep mode.  Currently saves about 20mA STM32F405 current
    // consumption (~30%).
    { __asm volatile ("wfi"); }
}

void vApplicationMallocFailedHook( void )
{
    fail("Malloc failed");
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char * pcTaskName)
{
    fail("Stack overflow");
}

/**
 * @brief configSUPPORT_STATIC_ALLOCATION is set to 1, so the application
 * must provide an implementation of vApplicationGetIdleTaskMemory() to
 * provide the memory that is
 * used by the Idle task.
 */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
        StackType_t **ppxIdleTaskStackBuffer,
        uint32_t *pulIdleTaskStackSize )
{
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

/*———————————————————–*/

/**
 * @brief configSUPPORT_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
 * application must provide an implementation of vApplicationGetTimerTaskMemory()
 * to provide the memory that is used by the Timer service task.
 */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer,
        StackType_t **ppxTimerTaskStackBuffer,
        uint32_t *pulTimerTaskStackSize )
{
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

void debugSendTraceInfo(unsigned int taskNbr)
{
}
