#include <hackflight.hpp>
#include "naze.hpp"

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

    SetSysClock(0);

    systemInit();

    hf::Hackflight h;
    h.init(new hf::Naze());

    while (true) {
        h.update();
    }
}
