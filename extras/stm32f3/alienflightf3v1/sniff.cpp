#include <f3board.h>
#include <Wire.h>
#include <hackflight.hpp>

void setup() {

    Wire.begin(2); 
}

void loop() {

    for (uint8_t addr=0; addr<128; ++addr) {
        Wire.beginTransmission(addr);
        if (!Wire.endTransmission()) {
            hf::Debug::printf("Found device at address 0X%02X\n", addr);
        }
    }

    hf::Debug::printf("--------------------------\n");

    delay(500);
}

extern "C" {

#include "serial_usb_vcp.h"

    serialPort_t * serial0_open(void)
    {
        return usbVcpOpen();
    }

}
