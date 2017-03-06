#include <hackflight.hpp>

#include <stm32f10x_conf.h>
#include <drv_gpio.h>
#include <drv_system.h>
#include <drv_serial.h>
#include <drv_uart.h>
#include <drv_serial.h>

#include "naze.hpp"

int main()
{
    hf::Hackflight h;
    h.init(new hf::Naze());

    while (true) {
        h.update();
    }
}
