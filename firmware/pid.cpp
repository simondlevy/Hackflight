extern "C" {

#include "mw.hpp"
#include "pid.hpp"

void pidCompute(
        int16_t rcCommand[4], 
        int16_t angle[3], 
        int16_t gyroADC[3],
        int16_t axisPID[3], 
        int32_t errorGyroI[3], 
        int32_t errorAngleI[3])
{
    static int16_t lastGyro[3] = { 0, 0, 0 };
    static int32_t delta1[3], delta2[3];

    int32_t PTermACC = 0, ITermACC = 0, PTermGYRO = 0, ITermGYRO = 0;

    // PITCH & ROLL & YAW PID
    int prop = max(abs(rcCommand[PITCH]), abs(rcCommand[ROLL])); // range [0;500]

    for (uint8_t axis = 0; axis < 3; axis++) {

        if (CONFIG_HORIZON_MODE && axis < 2) { // MODE relying on ACC

            // 50 degrees max inclination
            int32_t errorAngle = constrainer(2 * rcCommand[axis], -((int)CONFIG_MAX_ANGLE_INCLINATION), 
                    + CONFIG_MAX_ANGLE_INCLINATION) - angle[axis] + CONFIG_ANGLE_TRIM[axis];
            PTermACC = errorAngle * CONFIG_LEVEL_P / 100; 

            // 32 bits is needed for calculation: errorAngle*CONFIG_LEVEL_P could exceed 32768   
            // 16 bits is ok for result
            PTermACC = constrainer(PTermACC, -CONFIG_LEVEL_D * 5, + CONFIG_LEVEL_D * 5);

            errorAngleI[axis] = constrainer(errorAngleI[axis] + errorAngle, -10000, +10000); // WindUp
            ITermACC = (errorAngleI[axis] * CONFIG_LEVEL_I) >> 12;
        }

        if (CONFIG_HORIZON_MODE || axis == 2) { // MODE relying on GYRO or YAW axis

            int32_t error = (int32_t)rcCommand[axis] * 10 * 8 / CONFIG_AXIS_P[axis];
            error -= gyroADC[axis];

            PTermGYRO = rcCommand[axis];

            errorGyroI[axis] = constrainer(errorGyroI[axis] + error, -16000, +16000); // WindUp
            if ((abs(gyroADC[axis]) > 640) || ((axis == YAW) && (abs(rcCommand[axis]) > 100)))
                errorGyroI[axis] = 0;
            ITermGYRO = (errorGyroI[axis] / 125 * CONFIG_AXIS_I[axis]) >> 6;
        }

        int32_t PTerm = PTermGYRO;
        int32_t ITerm = ITermGYRO;

        if (CONFIG_HORIZON_MODE && axis < 2) {
            PTerm = (PTermACC * (500 - prop) + PTermGYRO * prop) / 500;
            ITerm = (ITermACC * (500 - prop) + ITermGYRO * prop) / 500;
        } 

        PTerm -= (int32_t)gyroADC[axis] * CONFIG_AXIS_P[axis] / 10 / 8; // 32 bits is needed for calculation
        int32_t delta = gyroADC[axis] - lastGyro[axis];
        lastGyro[axis] = gyroADC[axis];
        int32_t deltaSum = delta1[axis] + delta2[axis] + delta;
        delta2[axis] = delta1[axis];
        delta1[axis] = delta;
        int32_t DTerm = (deltaSum * CONFIG_AXIS_D[axis]) / 32;
        axisPID[axis] = PTerm + ITerm - DTerm;
    }
}

} // extern "C"
