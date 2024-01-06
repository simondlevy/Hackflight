#include <free_rtos.h>
#include <task.h>

#include "platform/platform.h"

#include <system.h>

#include "bootloader.h"
#include "led.h"

int main() 
{
    check_enter_bootloader();

    // Initialize the platform.
    if (platformInit() != 0) {

        // The firmware is running on the wrong hardware. Halt
        while(true) {
        }
    }

    // Launch the system task that will initialize and start everything
    systemLaunch();

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    ledInit();

    while(true) {
    }

    return 0;
}
