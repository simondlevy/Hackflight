/*
#include <hackflight.hpp>
#include "naze.hpp"

#include <stm32f10x_conf.h>
#include <drv_gpio.h>
#include <drv_system.h>
#include <drv_serial.h>
#include <drv_uart.h>
#include <drv_serial.h>
*/

int main(int argc, char ** argv)
{
    void SetSysClock(bool overclock);
    void systemInit(void);

    // Configure clock, this figures out HSE for hardware autodetect
    SetSysClock(0);

    systemInit();

    /*
    hf::Hackflight h;
    h.init(new hf::Naze());

    while (true) {
        h.update();
    }
    */
}
