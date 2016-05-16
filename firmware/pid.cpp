extern "C" {

#include "mw.hpp"

void PID::init(void)
{
    for (uint8_t axis=0; axis<3; ++axis) {
        this->lastGyro[axis] = 0;
        this->delta1[axis] = 0;
        this->delta2[axis] = 0;
    }

    this->resetIntegral();
}

void PID::update(RC * rc, IMU * imu)
{
    int32_t PTermACC = 0, ITermACC = 0, PTermGYRO = 0, ITermGYRO = 0;

    // PITCH & ROLL & YAW PID
    int prop = max(abs(rc->command[PITCH]), abs(rc->command[ROLL])); // range [0;500]

    for (uint8_t axis = 0; axis < 3; axis++) {

        if (axis < 2) { // MODE relying on ACC

            // 50 degrees max inclination
            int32_t errorAngle = constrain(2 * rc->command[axis], -((int)CONFIG_MAX_ANGLE_INCLINATION), 
                    + CONFIG_MAX_ANGLE_INCLINATION) - imu->angle[axis] + CONFIG_ANGLE_TRIM[axis];
            PTermACC = errorAngle * CONFIG_LEVEL_P / 100; 

            // 32 bits is needed for calculation: this->errorAngle*CONFIG_LEVEL_P could exceed 32768   
            // 16 bits is ok for result
            PTermACC = constrain(PTermACC, -CONFIG_LEVEL_D * 5, + CONFIG_LEVEL_D * 5);

            this->errorAngleI[axis] = constrain(this->errorAngleI[axis] + errorAngle, -10000, +10000); // WindUp
            ITermACC = (this->errorAngleI[axis] * CONFIG_LEVEL_I) >> 12;
        }

        int32_t error = (int32_t)rc->command[axis] * 10 * 8 / CONFIG_AXIS_P[axis];
        error -= imu->gyroADC[axis];

        PTermGYRO = rc->command[axis];

        this->errorGyroI[axis] = constrain(this->errorGyroI[axis] + error, -16000, +16000); // WindUp
        if ((abs(imu->gyroADC[axis]) > 640) || ((axis == YAW) && (abs(rc->command[axis]) > 100)))
            this->errorGyroI[axis] = 0;
        ITermGYRO = (this->errorGyroI[axis] / 125 * CONFIG_AXIS_I[axis]) >> 6;
        int32_t PTerm = PTermGYRO;
        int32_t ITerm = ITermGYRO;

        if (axis < 2) {
            PTerm = (PTermACC * (500 - prop) + PTermGYRO * prop) / 500;
            ITerm = (ITermACC * (500 - prop) + ITermGYRO * prop) / 500;
        } 

        PTerm -= (int32_t)imu->gyroADC[axis] * CONFIG_AXIS_P[axis] / 10 / 8; // 32 bits is needed for calculation
        int32_t delta = imu->gyroADC[axis] - this->lastGyro[axis];
        this->lastGyro[axis] = imu->gyroADC[axis];
        int32_t deltaSum = this->delta1[axis] + this->delta2[axis] + delta;
        this->delta2[axis] = this->delta1[axis];
        this->delta1[axis] = delta;
        int32_t DTerm = (deltaSum * CONFIG_AXIS_D[axis]) / 32;
        this->axisPID[axis] = PTerm + ITerm - DTerm;
    }

    // prevent "yaw jump" during yaw correction
    this->axisPID[YAW] = constrain(this->axisPID[YAW], -100 - abs(rc->command[YAW]), +100 + abs(rc->command[YAW]));
}

void PID::resetIntegral(void)
{
    this->errorGyroI[ROLL] = 0;
    this->errorGyroI[PITCH] = 0;
    this->errorGyroI[YAW] = 0;
    this->errorAngleI[ROLL] = 0;
    this->errorAngleI[PITCH] = 0;
}

} // extern "C"
