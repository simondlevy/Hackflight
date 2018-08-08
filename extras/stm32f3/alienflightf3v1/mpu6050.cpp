#include <f3board.h>
#include <Wire.h>
#include <hackflight.hpp>
#include <MPU6050.h>

MPU6050 * imu;

static const Gscale_t GSCALE = GFS_250DPS;
static const Ascale_t ASCALE = AFS_2G;

void setup() {

    Wire.begin(2); 

    imu = new MPU6050();

    imu->begin();

    imu->initMPU6050(ASCALE, GSCALE); 
}

void loop() {

    if (imu->checkNewData()) {  
        int16_t accelCount[3];           
        imu->readAccelData(accelCount);  
        hf::Debug::printf("%d %d %d\n", accelCount[0], accelCount[1], accelCount[2]);
    }
}

extern "C" {

#include "serial_usb_vcp.h"

    serialPort_t * serial0_open(void)
    {
        return usbVcpOpen();
    }

}
