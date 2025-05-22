#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

#include <motors.h>

static void fail(const char * msg)
{
    portDISABLE_INTERRUPTS();
    motorsStop();
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
