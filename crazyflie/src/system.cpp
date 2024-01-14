#include <free_rtos.h>
#include <semphr.h>
#include <task.h>

#include <crossplatform.h>

#include <configblock.hpp>

#include <radiolink.hpp>
#include <storage.h>

#include <arduino/time.h>

#include <hal/i2cdev.h>
#include <hal/uart_syslink.h>
#include <hal/watchdog.h>

#include <crtp/crtp.h>
#include <crtp/crtp_mem.hpp>

#include <mixers/quadrotor.hpp>

#include <platform/platform.h>

// Crazyfle tasks
#include <tasks/log.h>
#include <tasks/power.hpp>
#include <tasks/syslink.hpp>
#include <tasks/usblink.hpp>

// Cross-platform tasks
#include <tasks/core.hpp>
#include <tasks/estimator.hpp>
#include <tasks/flowdeck.hpp>
#include <tasks/imu.hpp>
#include <tasks/zranger.hpp>

#include <commander.hpp>
#include <config.h>
#include <console.h>
#include <led.h>
#include <mem.hpp>
#include <params.h>
#include <pinmap.h>
#include <safety.hpp>
#include <sysload.h>
#include <system.h>
#include <worker.hpp>

// Arduino class
#include <vl53l1.hpp>

#define __main
#include <Arduino.h>

static const uint8_t VL53L1_DEFAULT_ADDRESS = 0x29;
static const uint8_t VL53L1_NEW_ADDRESS     = 0x31;

// Globals -------------------------------------------------------------------

Safety safety;

CoreTask coreTask;
EstimatorTask estimatorTask;
FlowDeckTask flowDeckTask;
ZRangerTask zrangerTask;

Commander commander;
PowerMonitorTask powerMonitorTask;
Worker worker;

RadioLink radioLink;
UsbLinkTask usbLinkTask;

PowerMonitorTask::syslinkInfo_t pmSyslinkInfo;

// ---------------------------------------------------------------------------

static ImuTask imuTask;

static ConfigBlock configBlock;

typedef enum {
    linkEcho   = 0x00,
    linkSource = 0x01,
    linkSink   = 0x02,
} LinkNbr;

// Shared with params
uint16_t system_echoDelay;

static bool paramDidInit;

static VL53L1 vl53l1;

static void paramTask(void * prm)
{
    (void)prm;
    runParamTask();
}

static void crtpSrvTask(void* prm)
{
    static crtpPacket_t p;

    crtpInitTaskQueue(CRTP_PORT_LINK);

    while(1) {
        crtpReceivePacketBlock(CRTP_PORT_LINK, &p);

        switch (p.channel)
        {
            case linkEcho:
                if (system_echoDelay > 0) {
                    vTaskDelay((uint32_t)system_echoDelay);
                }
                crtpSendPacketBlock(&p);
                break;
            case linkSource:
                p.size = CRTP_MAX_DATA_SIZE;
                bzero(p.data, CRTP_MAX_DATA_SIZE);
                strcpy((char*)p.data, "Bitcraze Crazyflie");
                crtpSendPacketBlock(&p);
                break;
            case linkSink:
                /* Ignore packet */
                break;
            default:
                break;
        }
    }
}

static void memTask(void* param) 
{
    crtpInitTaskQueue(CRTP_PORT_MEM);

    // This should be synced with decks starting up, otherwise
    // there might be late arrivals for the registration that will
    // trigger assert.

    systemWaitStart();

    // Do not allow registration of new handlers after this point as clients now can start
    // to query for available memories
    memBlockHandlerRegistration();

    CrtpMem::run();
}


static bool selftestPassed;
static uint8_t dumpAssertInfo = 0;
static bool didInit;
static bool crtpMemDidInit;
static bool crtpServiceDidInit;

static char nrf_version[16];

xSemaphoreHandle canStartMutex;
static StaticSemaphore_t canStartMutexBuffer;

static const auto CRTP_TASK_STACK_DEPTH = configMINIMAL_STACK_SIZE;
StackType_t  crtpTaskStackBuffer[CRTP_TASK_STACK_DEPTH]; 
StaticTask_t crtpTaskTaskBuffer;

static void crtpServiceInit(void)
{
    if (crtpServiceDidInit) {
        return;
    }

    xTaskCreateStatic(
            crtpSrvTask, 
            "CRTP-SRV", 
            CRTP_TASK_STACK_DEPTH, 
            NULL, 
            0, 
            crtpTaskStackBuffer,
            &crtpTaskTaskBuffer);

    crtpServiceDidInit = true;
}

static const auto PARAM_TASK_STACK_DEPTH = configMINIMAL_STACK_SIZE;
StackType_t  paramTaskStackBuffer[PARAM_TASK_STACK_DEPTH]; 
StaticTask_t paramTaskTaskBuffer;

static void paramInit(void)
{
    xTaskCreateStatic(
            paramTask, 
            "PARAM", 
            PARAM_TASK_STACK_DEPTH,
            NULL, 
            1, 
            paramTaskStackBuffer,
            &paramTaskTaskBuffer);

    paramDidInit = true;
}

static void commInit(void)
{
    uartslkInit();
    radioLink.init(configBlock);

    crtpSetLink(CRTP_LINK_RADIO);

    crtpServiceInit();

    platformServiceInit(&safety);

    logInit();

    paramInit();
}

static bool systemTest()
{
    bool pass=didInit;

    pass &= ledseqTest();
    pass &= powerMonitorTask.test();
    pass &= worker.test();
    return pass;
}

static void systemStart()
{
    xSemaphoreGive(canStartMutex);
    watchdogInit();
}

static void systemRequestNRFVersion()
{
    syslinkPacket_t slp;

    slp.type = SYSLINK_SYS_NRF_VERSION;
    slp.length = 0;
    syslinkSendPacket(&slp);
}

static bool commTest(void)
{
    bool pass = true;

    pass &= radioLink.test();
    pass &= crtpTest();
    pass &= crtpServiceDidInit;
    pass &= platformServiceTest();
    pass &= consoleTest();
    pass &= paramDidInit;

    return pass;
}

static const auto MEM_TASK_STACK_DEPTH = configMINIMAL_STACK_SIZE;
StackType_t  memTaskStackBuffer[MEM_TASK_STACK_DEPTH]; 
StaticTask_t memTaskTaskBuffer;

static void getOpenLoopDemands(
        demands_t & demands, uint32_t & timestamp, bool & inHoverMode)
{
    commander.getDemands(demands, timestamp, inHoverMode);
}

static void systemTask(void *arg)
{
    bool pass = true;

    i2cdevInit(I2C3_DEV);
    i2cdevInit(I2C1_DEV);
    analogInit();
    ledInit();
    ledSet(CHG_LED, 1);
    usecTimerInit();

    if(didInit)
        return;

    canStartMutex = xSemaphoreCreateMutexStatic(&canStartMutexBuffer);
    xSemaphoreTake(canStartMutex, portMAX_DELAY);

    usbLinkTask.begin();
    sysLoadInit();

    crtpInit();
    consoleInit();

    consolePrintf("SYSTEM: ----------------------------\n");
    consolePrintf("SYSTEM: %s is up and running!\n", platformConfigGetDeviceTypeName());

    consolePrintf("SYSTEM: I am 0x%08X%08X%08X and I have %dKB of flash!\n",
            *((int*)(MCU_ID_ADDRESS+8)), *((int*)(MCU_ID_ADDRESS+4)),
            *((int*)(MCU_ID_ADDRESS+0)), *((short*)(MCU_FLASH_SIZE_ADDRESS)));

    configBlock.init();
    storageInit();
    worker.init();
    ledseqInit();
    powerMonitorTask.begin(pmSyslinkInfo, &worker);

    didInit = true;

    commInit();

    commander.init();

    xTaskCreateStatic(
            memTask, 
            "MEM", 
            MEM_TASK_STACK_DEPTH,
            NULL, 
            1, 
            memTaskStackBuffer,
            &memTaskTaskBuffer);

    crtpMemDidInit = true;

    // Enabling incoming syslink messages to be added to the queue.
    // This should probably be done later, but deckInit() takes a long time if
    // this is done later.
    uartslkEnableIncoming();

    memInit();

    SPI.begin();

    vl53l1.init(I2C1_DEV, VL53L1_DEFAULT_ADDRESS);

    if (vl53l1.changeAddress(VL53L1_DEFAULT_ADDRESS, VL53L1_NEW_ADDRESS) && 
            vl53l1.begin()) {

        consolePrintf("ZRANGER: Z-down sensor [OK]\n");
    }
    else {
        consolePrintf("ZRANGER: Z-down sensor [FAIL]\n");
    }

    // Launch the cross-platform tasks ---------------------------------------

    estimatorTask.begin(&safety);

    flowDeckTask.begin(PIN_FLOWDECK_CS, &estimatorTask);

    zrangerTask.begin(&vl53l1, &estimatorTask);

    imuTask.begin(
            &estimatorTask, 
            configBlock.getCalibRoll(), 
            configBlock.getCalibPitch());

    auto coreTaskReady = coreTask.begin(
            &safety,
            &estimatorTask,
            &imuTask,
            getOpenLoopDemands,
            mixQuadrotor);

    // -----------------------------------------------------------------------

    systemRequestNRFVersion();

    //Test the modules
    consolePrintf("SYSTEM: About to run tests in system.c.\n");
    if (systemTest() == false) {
        pass = false;
        consolePrintf("SYSTEM: system [FAIL]\n");
    }
    if (configBlock.test() == false) {
        pass = false;
        consolePrintf("SYSTEM: configblock [FAIL]\n");
    }
    if (storageTest() == false) {
        pass = false;
        consolePrintf("SYSTEM: storage [FAIL]\n");
    }
    if (commTest() == false) {
        pass = false;
        consolePrintf("SYSTEM: comm [FAIL]\n");
    }
    if (commander.test() == false) {
        pass = false;
        consolePrintf("SYSTEM: commander [FAIL]\n");
    }

    if (!coreTaskReady) {
        pass = false;
        consolePrintf("SYSTEM: core task [FAIL]\n");
    }

    if (!crtpMemDidInit) {
        pass = false;
        consolePrintf("SYSTEM: CRTP mem [FAIL]\n");
    }
    if (watchdogNormalStartTest() == false) {
        pass = false;
        consolePrintf("SYSTEM: watchdogNormalStart [FAIL]\n");
    }
    if (cfAssertNormalStartTest() == false) {
        pass = false;
        consolePrintf("SYSTEM: cfAssertNormalStart [FAIL]\n");
    }

    //Start the firmware
    if(pass) {
        consolePrintf("SYSTEM: Self test passed!\n");
        selftestPassed = 1;
        systemStart();

        ledseqShowSuccess();

    }

    else {
        selftestPassed = 0;
        if (systemTest())
        {
            while(1) {

                ledseqShowFailure();


                vTaskDelay(M2T(2000));
                // System can be forced to start by setting the param to 1 from the cfclient
                if (selftestPassed)
                {
                    consolePrintf("SYSTEM: Start forced.\n");
                    systemStart();
                    break;
                }
            }
        }
        else {

            ledInit();

            ledShowSys();
        }
    }

    worker.loop();

    //Should never reach this point!
    while(1)
        vTaskDelay(portMAX_DELAY);
}



//////////////////////////////////////////////////////////////////////////////

void systemLaunch(void)
{
    xTaskCreate(
            systemTask, 
            "SYSTEM", 2* configMINIMAL_STACK_SIZE, 
            NULL, 
            2, 
            NULL);
}

void systemWaitStart(void)
{
    // This guarantees that the system task is initialized before other
    // tasks waits for the start event.
    while(!didInit) {
        vTaskDelay(2);
    }

    xSemaphoreTake(canStartMutex, portMAX_DELAY);
    xSemaphoreGive(canStartMutex);
}

void systemSyslinkReceive(syslinkPacket_t *slp)
{
    if (slp->type == SYSLINK_SYS_NRF_VERSION)
    {
        size_t len = slp->length - 1;

        if (sizeof(nrf_version) - 1 <=  len) {
            len = sizeof(nrf_version) - 1;
        }
        memcpy(&nrf_version, &slp->data[0], len );
        consolePrintf("SYSTEM: NRF51 version: %s\n", nrf_version);
    }
}

extern "C" {

    void vApplicationIdleHook( void )
    {
        static uint32_t tickOfLatestWatchdogReset = M2T(0);

        portTickType tickCount = xTaskGetTickCount();

        if (tickCount - tickOfLatestWatchdogReset > M2T(WATCHDOG_RESET_PERIOD_MS))
        {
            tickOfLatestWatchdogReset = tickCount;
            watchdogReset();
        }

        if (dumpAssertInfo != 0) {
            printAssertSnapshotData();
            dumpAssertInfo = 0;
        }

        // Enter sleep mode. Does not work when debugging chip with SWD.
        // Currently saves about 20mA STM32F405 current consumption (~30%).
        { __asm volatile ("wfi"); }
    }

    void vApplicationMallocFailedHook( void )
    {
        portDISABLE_INTERRUPTS();
        consolePrintf("SYSTEM: \nMalloc failed!\n");

        ledShowFailure();

        motorsStop();
        storeAssertTextData("Malloc failed");
        while(1);
    }

    void vApplicationStackOverflowHook(TaskHandle_t xTask, char * pcTaskName)
    {
        portDISABLE_INTERRUPTS();
        consolePrintf("SYSTEM: \nStack overflow!\n");

        ledShowFailure();

        motorsStop();
        storeAssertTextData("Stack overflow");
        while(1);
    }

    /**
     * @brief configSUPPORT_STATIC_ALLOCATION is set to 1, so the application must provide an
     * implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
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

} // extern "C"

