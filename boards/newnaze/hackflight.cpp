extern "C" {

#include <stm32f10x_conf.h>
#include <drv_gpio.h>
#include <drv_system.h>
#include <drv_serial.h>
#include <drv_uart.h>
#include <drv_serial.h>

#include <stdlib.h>

int main(void)
{
    void SetSysClock(bool overclock);
    void systemInit(void);

    // Configure clock, this figures out HSE for hardware autodetect
    SetSysClock(0);

    systemInit();

    while (true) {
    }
}

} // extern "C"


