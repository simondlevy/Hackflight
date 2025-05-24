// clang-format off

/*
 * FreeRTOS Kernel V11.0.1
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */


#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * See http://www.freertos.org/a00110.html
 *----------------------------------------------------------*/


#define configUSE_PREEMPTION                        1
#define configUSE_TICKLESS_IDLE                     0
#define configCPU_CLOCK_HZ                          ( F_CPU )
#define configSYSTICK_CLOCK_HZ                      ( 100000UL )
#define configTICK_RATE_HZ                          ( (TickType_t) 1000 )
#define configUSE_PORT_OPTIMISED_TASK_SELECTION     1
#define configMAX_PRIORITIES                        ( 10 )
#define configMINIMAL_STACK_SIZE                    ( ( unsigned short ) 128 )
#define configMAX_TASK_NAME_LEN                     ( 10 )
#define configTICK_TYPE_WIDTH_IN_BITS               TICK_TYPE_WIDTH_32_BITS
#define configIDLE_SHOULD_YIELD                     1
#define configUSE_TASK_NOTIFICATIONS                1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES       4
#define configUSE_MUTEXES                           1
#define configUSE_RECURSIVE_MUTEXES                 1
#define configUSE_COUNTING_SEMAPHORES               1
#define configQUEUE_REGISTRY_SIZE                   0
#define configUSE_QUEUE_SETS                        0
#define configUSE_TIME_SLICING                      0
#define configUSE_NEWLIB_REENTRANT                  1
#define configENABLE_BACKWARD_COMPATIBILITY         0
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS     4
#define configUSE_APPLICATION_TASK_TAG              0

/* Tasks.c additions (e.g. Thread Aware Debug capability) */
#define configINCLUDE_FREERTOS_TASK_C_ADDITIONS_H   1

/* Memory allocation related definitions. */
#define configSUPPORT_STATIC_ALLOCATION             1
#define configSUPPORT_DYNAMIC_ALLOCATION            1
#define configAPPLICATION_ALLOCATED_HEAP            0

/* Hook function related definitions. */
#define configUSE_IDLE_HOOK                         1
#define configUSE_TICK_HOOK                         1
#define configCHECK_FOR_STACK_OVERFLOW              2
#define configUSE_MALLOC_FAILED_HOOK                1
#define configUSE_DAEMON_TASK_STARTUP_HOOK          0

/* Run time stats gathering definitions. */
#define configGENERATE_RUN_TIME_STATS               1
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()
#define portGET_RUN_TIME_COUNTER_VALUE()            freertos_get_us()
#define configUSE_TRACE_FACILITY                    1
#define configUSE_STATS_FORMATTING_FUNCTIONS        0

/* Task aware debugging. */
#define configRECORD_STACK_HIGH_ADDRESS             1

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES                       0
#define configMAX_CO_ROUTINE_PRIORITIES             ( 2 )

/* Software timer definitions. */
#define configUSE_TIMERS                            1
#define configTIMER_TASK_PRIORITY                   ( configMAX_PRIORITIES - 1 )
#define configTIMER_QUEUE_LENGTH                    10
#ifndef configTIMER_TASK_STACK_DEPTH
#define configTIMER_TASK_STACK_DEPTH                ( 1536U / 4U )
#endif
#define configIDLE_TASK_NAME                        "IDLE"

/* Define to trap errors during development. */
#ifdef NDEBUG
#define configCHECK_HANDLER_INSTALLATION            0
#define configUSE_LIST_DATA_INTEGRITY_CHECK_BYTES   0
#define configASSERT(condition) ((void) 0)
#define putchar_debug(...)
#define printf_debug(...)
#define ASSERT_LOG(...)
#else
#define configCHECK_HANDLER_INSTALLATION            1
#define configUSE_LIST_DATA_INTEGRITY_CHECK_BYTES   1
#ifdef __cplusplus
extern "C" {
#endif
void assert_blink(const char*, int, const char*, const char*) __attribute__((noreturn));
#ifdef __cplusplus
}
#define ASSERT_LOG(_msg) assert_blink("", __LINE__, __PRETTY_FUNCTION__, #_msg);
#else
#if defined ARDUINO_TEENSY40 || defined ARDUINO_TEENSY41
#define PROGMEM_FREERTOS __attribute__((section(".progmem")))
#else
#define PROGMEM_FREERTOS
#endif
#define ASSERT_LOG(_msg)                                                            \
    {                                                                               \
        static const char _file_[] PROGMEM_FREERTOS = __FILE__;                     \
        assert_blink((const char*) _file_, __LINE__, __PRETTY_FUNCTION__, #_msg);   \
    }
#endif // __cplusplus
#define configASSERT(_e)               \
    if (__builtin_expect(!!(_e), 1)) { \
        (void) 0;                      \
    } else {                           \
        ASSERT_LOG(_e);                \
    }
#ifdef PRINT_DEBUG_STUFF
void putchar_debug(char);
void printf_debug(const char*, ...);
#else
#define putchar_debug(...)
#define printf_debug(...)
#endif // PRINT_DEBUG_STUFF
#endif // NDEBUG

#if configGENERATE_RUN_TIME_STATS == 1
uint64_t freertos_get_us(void);
#endif

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */
#define INCLUDE_vTaskPrioritySet                    1
#define INCLUDE_uxTaskPriorityGet                   1
#define INCLUDE_vTaskDelete                         1
#define INCLUDE_vTaskCleanUpResources               1
#define INCLUDE_vTaskSuspend                        1
#define INCLUDE_xTaskDelayUntil                     1
#define INCLUDE_vTaskDelay                          1
#define INCLUDE_eTaskGetState                       1
#define INCLUDE_xTimerPendFunctionCall              1
#define INCLUDE_xSemaphoreGetMutexHolder            0
#define INCLUDE_xTaskGetSchedulerState              1
#define INCLUDE_xTaskGetCurrentTaskHandle           1
#define INCLUDE_uxTaskGetStackHighWaterMark         1
#define INCLUDE_xTaskGetIdleTaskHandle              1
#define INCLUDE_eTaskGetState                       1
#define INCLUDE_xTaskAbortDelay                     1
#define INCLUDE_xTaskGetHandle                      1
#define INCLUDE_xTaskResumeFromISR                  1

/* Cortex-M specific definitions. */
#ifdef __NVIC_PRIO_BITS
    /* __BVIC_PRIO_BITS will be specified when CMSIS is being used. */
    #define configPRIO_BITS                         __NVIC_PRIO_BITS
#else
    #define configPRIO_BITS                         4 /* 15 priority levels */
#endif

/* The lowest interrupt priority that can be used in a call to a "set priority"
function. */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY     ( ( 1U << ( configPRIO_BITS ) ) - 1 )

/* The highest interrupt priority that can be used by any interrupt service
routine that makes calls to interrupt safe FreeRTOS API functions.  DO NOT CALL
INTERRUPT SAFE FREERTOS API FUNCTIONS FROM ANY INTERRUPT THAT HAS A HIGHER
PRIORITY THAN THIS! (higher priorities are lower numeric values. */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY    2

/* Interrupt priorities used by the kernel port layer itself.  These are generic
to all Cortex-M ports, and do not rely on any particular library functions. */
#define configKERNEL_INTERRUPT_PRIORITY             ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY        ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )

#define configUSE_GCC_BUILTIN_ATOMICS               1

#ifdef __cplusplus
}
#endif

#if defined(__has_include) && __has_include("freertos_config_override.h")
// config override does not work if used as an Arduino library with Teensyduino
#include "freertos_config_override.h"
#endif

#endif /* FREERTOS_CONFIG_H */
