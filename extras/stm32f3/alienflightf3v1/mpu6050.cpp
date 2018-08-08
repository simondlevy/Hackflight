#include <f3board.h>
#include <Wire.h>
#include <hackflight.hpp>
#include <MPU6050.h>

MPU6050 * imu;

void setup() {

    Wire.begin(2); 

    imu = new MPU6050();

    imu->begin();
}

void loop() {

    uint8_t c = imu->getMPU6050ID();
    hf::Debug::printf("%x\n", c);
    delay(500);
}

extern "C" {

#include "serial_usb_vcp.h"

    serialPort_t * serial0_open(void)
    {
        return usbVcpOpen();
    }

}
