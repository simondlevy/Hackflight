extern "C" {

#include "mw.hpp"
#include "imu.hpp"

    void IMU::init(uint16_t *acc1G, float * gyroScale) {

        board_imuInit(acc1G, gyroScale);
    }

}
