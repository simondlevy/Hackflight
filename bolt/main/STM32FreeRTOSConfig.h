/*
 * FreeRTOSConfig.h for hackflight Bolt
 *
 * Copyright (C) 2025 Simon D. Levy
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
 */

#pragma once

#include "FreeRTOSConfig_Default.h"

#include <Arduino.h>

#define usecTimerInit()

#define FREERTOS_MCU_CLOCK_HZ   168000000

#define configGENERATE_RUN_TIME_STATS 1
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() usecTimerInit()
#define portGET_RUN_TIME_COUNTER_VALUE() micros()

#define configGENERATE_RUN_TIME_STATS 1
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() usecTimerInit()
#define portGET_RUN_TIME_COUNTER_VALUE() micros()

#define FREERTOS_HEAP_SIZE      30000
#define FREERTOS_MIN_STACK_SIZE 150       

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * See http://www.freertos.org/a00110.html.
 *----------------------------------------------------------*/

#define configUSE_PREEMPTION		1
#define configUSE_IDLE_HOOK			1
#define configUSE_TICK_HOOK			0
#define configCPU_CLOCK_HZ			( ( unsigned long ) FREERTOS_MCU_CLOCK_HZ )
#define configTICK_RATE_HZ_RAW  1000
#define configTICK_RATE_HZ			( ( TickType_t ) configTICK_RATE_HZ_RAW )
#define configMINIMAL_STACK_SIZE	( ( unsigned short ) FREERTOS_MIN_STACK_SIZE )
#define configTOTAL_HEAP_SIZE		( ( size_t ) ( FREERTOS_HEAP_SIZE ) )
#define configMAX_TASK_NAME_LEN		( 10 )
#define configUSE_16_BIT_TICKS		0
#define configIDLE_SHOULD_YIELD		0
#define configUSE_CO_ROUTINES 		0
#define configCHECK_FOR_STACK_OVERFLOW      1
#define configUSE_TASK_NOTIFICATIONS 1
#define configUSE_TIMERS          1
#define configTIMER_TASK_PRIORITY 1
#define configTIMER_QUEUE_LENGTH  20
#define configUSE_MALLOC_FAILED_HOOK 1
#define configTIMER_TASK_STACK_DEPTH (configMINIMAL_STACK_SIZE * 4)

#define configMAX_PRIORITIES		( 6 )
#define configMAX_CO_ROUTINE_PRIORITIES ( 2 )

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */

#define INCLUDE_vTaskPrioritySet		1
#define INCLUDE_uxTaskPriorityGet		1
#define INCLUDE_vTaskDelete				1
#define INCLUDE_vTaskCleanUpResources	1
#define INCLUDE_vTaskSuspend			1
#define INCLUDE_vTaskDelayUntil			1
#define INCLUDE_vTaskDelay				1
#define INCLUDE_uxTaskGetStackHighWaterMark 1
#define INCLUDE_xTaskGetIdleTaskHandle 1
#define INCLUDE_xTimerPendFunctionCall 1

#define configUSE_MUTEXES 1

#define configKERNEL_INTERRUPT_PRIORITY     255
//#define configMAX_SYSCALL_INTERRUPT_PRIORITY 1
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 0x5F /* equivalent to 0x05, or priority 5. */

//Map the port handler to the crt0 interruptions handlers
#define xPortPendSVHandler PendSV_Handler
#define xPortSysTickHandler tickFreeRTOS
#define vPortSVCHandler SVC_Handler

#define M2T(X) ((unsigned int)(X))
#define T2M(X) ((unsigned int)(X))
#define F2T(X) ((unsigned int)((configTICK_RATE_HZ/(X))))

// Seconds to OS ticks
#define S2T(X) ((TickType_t)((X) * configTICK_RATE_HZ))
#define T2S(X) ((X) / (float)configTICK_RATE_HZ)

// DEBUG SECTION
#define configUSE_APPLICATION_TASK_TAG  1
#define configQUEUE_REGISTRY_SIZE       10

#define configSUPPORT_STATIC_ALLOCATION 1
