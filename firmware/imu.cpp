extern "C" {

#include <math.h>

#include "mw.hpp"
#include "imu.hpp"

#define INV_GYR_CMPF_FACTOR   (1.0f / ((float)CONFIG_GYRO_CMPF_FACTOR + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / ((float)CONFIG_GYRO_CMPFM_FACTOR + 1.0f))


typedef struct stdev_t {
    float m_oldM, m_newM, m_oldS, m_newS;
    int m_n;
} stdev_t;

static void devClear(struct stdev_t *dev)
{
    dev->m_n = 0;
}

static void devPush(stdev_t *dev, float x)
{
    dev->m_n++;
    if (dev->m_n == 1) {
        dev->m_oldM = dev->m_newM = x;
        dev->m_oldS = 0.0f;
    } else {
        dev->m_newM = dev->m_oldM + (x - dev->m_oldM) / dev->m_n;
        dev->m_newS = dev->m_oldS + (x - dev->m_oldM) * (x - dev->m_newM);
        dev->m_oldM = dev->m_newM;
        dev->m_oldS = dev->m_newS;
    }
}

static float devVariance(stdev_t *dev)
{
    return ((dev->m_n > 1) ? dev->m_newS / (dev->m_n - 1) : 0.0f);
}

static float devStandardDeviation(stdev_t *dev)
{
    return sqrtf(devVariance(dev));
}


// Normalize a vector
static void normalizeV(float src[3], float dest[3])
{
    float length = sqrtf(src[X] * src[X] + src[Y] * src[Y] + src[Z] * src[Z]);

    if (length != 0) {
        dest[X] = src[X] / length;
        dest[Y] = src[Y] / length;
        dest[Z] = src[Z] / length;
    }
}

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
static void rotateV(float v[3], float *delta)
{
    float * v_tmp = v;

    // This does a  "proper" matrix rotation using gyro deltas without small-angle approximation
    float mat[3][3];
    float cosx, sinx, cosy, siny, cosz, sinz;
    float coszcosx, sinzcosx, coszsinx, sinzsinx;

    cosx = cosf(delta[ROLL]);
    sinx = sinf(delta[ROLL]);
    cosy = cosf(delta[PITCH]);
    siny = sinf(delta[PITCH]);
    cosz = cosf(delta[YAW]);
    sinz = sinf(delta[YAW]);

    coszcosx = cosz * cosx;
    sinzcosx = sinz * cosx;
    coszsinx = sinx * cosz;
    sinzsinx = sinx * sinz;

    mat[0][0] = cosz * cosy;
    mat[0][1] = -cosy * sinz;
    mat[0][2] = siny;
    mat[1][0] = sinzcosx + (coszsinx * siny);
    mat[1][1] = coszcosx - (sinzsinx * siny);
    mat[1][2] = -sinx * cosy;
    mat[2][0] = (sinzsinx) - (coszcosx * siny);
    mat[2][1] = (coszsinx) + (sinzcosx * siny);
    mat[2][2] = cosy * cosx;

    v[X] = v_tmp[X] * mat[0][0] + v_tmp[Y] * mat[1][0] + v_tmp[Z] * mat[2][0];
    v[Y] = v_tmp[X] * mat[0][1] + v_tmp[Y] * mat[1][1] + v_tmp[Z] * mat[2][1];
    v[Z] = v_tmp[X] * mat[0][2] + v_tmp[Y] * mat[1][2] + v_tmp[Z] * mat[2][2];
}

static int32_t applyDeadband(int32_t value, int32_t deadband)
{
    if (abs(value) < deadband) {
        value = 0;
    } else if (value > 0) {
        value -= deadband;
    } else if (value < 0) {
        value += deadband;
    }
    return value;
}

void IMU::init(void) {

    board_imuInit(&this->acc1G, &this->gyroScale);

    // calculate RC time constant used in the accZ lpf    
    this->fcAcc = 0.5f / (M_PI * CONFIG_ACCZ_LPF_CUTOFF); 

    this->gyroADC[0] = 0;
    this->gyroADC[1] = 0;
    this->gyroADC[2] = 0;
}

void IMU::getEstimatedAttitude(bool armed, float anglerad[3], uint16_t & calibratingA, uint16_t & calibratingG)
{
    static float    accLPF[3];
    static int32_t  accZoffset;
    static float    accz_smooth;
    static int16_t  accZero[3];
    static int32_t  a[3];
    static int16_t  accSmooth[3];
    static int32_t  accSum[3];
    static uint32_t accTimeSum;        // keep track for integration of acc
    static float    EstG[3];
    static float    EstN[3] = { 1.0f, 0.0f, 0.0f };
    static int16_t  gyroZero[3];
    static uint32_t previousT;

    int32_t accMag = 0;
    float dT = 0;
    float rpy[3];
    float accel_ned[3];
    uint32_t currentT = board_getMicros();
    float deltaGyroAngle[3];
    uint32_t deltaT = currentT - previousT;
    float scale = deltaT * this->gyroScale;
    int16_t  accADC[3];

    previousT = currentT;

    board_imuRead(accADC, this->gyroADC);

    if (calibratingA > 0) {

        for (uint8_t axis = 0; axis < 3; axis++) {
            // Reset a[axis] at start of calibration
            if (calibratingA == CONFIG_CALIBRATING_ACC_CYCLES)
                a[axis] = 0;
            // Sum up CONFIG_CALIBRATING_ACC_CYCLES readings
            a[axis] += accADC[axis];
            // Clear global variables for next reading
            accADC[axis] = 0;
            accZero[axis] = 0;
        }
        // Calculate average, shift Z down by acc1G
        if (calibratingA == 1) {
            accZero[ROLL] = (a[ROLL] + (CONFIG_CALIBRATING_ACC_CYCLES / 2)) / CONFIG_CALIBRATING_ACC_CYCLES;
            accZero[PITCH] = (a[PITCH] + (CONFIG_CALIBRATING_ACC_CYCLES / 2)) / CONFIG_CALIBRATING_ACC_CYCLES;
            accZero[YAW] = (a[YAW] + (CONFIG_CALIBRATING_ACC_CYCLES / 2)) / CONFIG_CALIBRATING_ACC_CYCLES - this->acc1G;
        }
        calibratingA--;
    }

    accADC[ROLL] -= accZero[ROLL];
    accADC[PITCH] -= accZero[PITCH];
    accADC[YAW] -= accZero[YAW];

    // range: +/- 8192; +/- 2000 deg/sec

    static int32_t g[3];
    static stdev_t var[3];

    if (calibratingG > 0) {
        for (uint8_t axis = 0; axis < 3; axis++) {
            // Reset g[axis] at start of calibration
            if (calibratingG == CONFIG_CALIBRATING_GYRO_CYCLES) {
                g[axis] = 0;
                devClear(&var[axis]);
            }
            // Sum up 1000 readings
            g[axis] += this->gyroADC[axis];
            devPush(&var[axis], this->gyroADC[axis]);
            // Clear global variables for next reading
            this->gyroADC[axis] = 0;
            gyroZero[axis] = 0;
            if (calibratingG == 1) {
                float dev = devStandardDeviation(&var[axis]);
                // check deviation and startover if idiot was moving the model
                if (CONFIG_MORON_THRESHOLD && dev > CONFIG_MORON_THRESHOLD) {
                    calibratingG = CONFIG_CALIBRATING_GYRO_CYCLES;
                    devClear(&var[0]);
                    devClear(&var[1]);
                    devClear(&var[2]);
                    g[0] = g[1] = g[2] = 0;
                    continue;
                }
                gyroZero[axis] = (g[axis] + (CONFIG_CALIBRATING_GYRO_CYCLES / 2)) / CONFIG_CALIBRATING_GYRO_CYCLES;
            }
        }
        calibratingG--;
    }

    for (uint8_t axis = 0; axis < 3; axis++)
        this->gyroADC[axis] -= gyroZero[axis];

    // Initialization
    for (uint8_t axis = 0; axis < 3; axis++) {
        deltaGyroAngle[axis] = this->gyroADC[axis] * scale;
        if (CONFIG_ACC_LPF_FACTOR > 0) {
            accLPF[axis] = accLPF[axis] * (1.0f - (1.0f / CONFIG_ACC_LPF_FACTOR)) + accADC[axis] * 
                (1.0f / CONFIG_ACC_LPF_FACTOR);
            accSmooth[axis] = accLPF[axis];
        } else {
            accSmooth[axis] = accADC[axis];
        }
        accMag += (int32_t)accSmooth[axis] * accSmooth[axis];
    }
    accMag = accMag * 100 / ((int32_t)this->acc1G * this->acc1G);

    rotateV(EstG, deltaGyroAngle);

    // Apply complimentary filter (Gyro drift correction)
    // If accel magnitude >1.15G or <0.85G and ACC vector outside of the limit
    // range => we neutralize the effect of accelerometers in the angle
    // estimation.  To do that, we just skip filter, as EstV already rotated by Gyro
    if (72 < (uint16_t)accMag && (uint16_t)accMag < 133) {
        for (uint8_t axis = 0; axis < 3; axis++)
            EstG[axis] = (EstG[axis] * (float)CONFIG_GYRO_CMPF_FACTOR + accSmooth[axis]) * INV_GYR_CMPF_FACTOR;
    }

    // Attitude of the estimated vector
    anglerad[ROLL] = atan2f(EstG[Y], EstG[Z]);
    anglerad[PITCH] = atan2f(-EstG[X], sqrtf(EstG[Y] * EstG[Y] + EstG[Z] * EstG[Z]));

    rotateV(EstN, deltaGyroAngle);
    normalizeV(EstN, EstN);

    // Calculate heading
    float cosineRoll = cosf(anglerad[ROLL]);
    float sineRoll = sinf(anglerad[ROLL]);
    float cosinePitch = cosf(anglerad[PITCH]);
    float sinePitch = sinf(anglerad[PITCH]);
    float Xh = EstN[X] * cosinePitch + EstN[Y] * sineRoll * sinePitch + EstN[Z] * sinePitch * cosineRoll;
    float Yh = EstN[Y] * cosineRoll - EstN[Z] * sineRoll;
    anglerad[YAW] = atan2f(Yh, Xh); 

    // deltaT is measured in us ticks
    dT = (float)deltaT * 1e-6f;

    // the accel values have to be rotated into the earth frame
    rpy[0] = -(float)anglerad[ROLL];
    rpy[1] = -(float)anglerad[PITCH];
    rpy[2] = -(float)anglerad[YAW];

    accel_ned[X] = accSmooth[0];
    accel_ned[Y] = accSmooth[1];
    accel_ned[Z] = accSmooth[2];

    rotateV(accel_ned, rpy);

    if (!armed) {
        accZoffset -= accZoffset / 64;
        accZoffset += accel_ned[Z];
    }
    accel_ned[Z] -= accZoffset / 64;  // compensate for gravitation on z-axis

    accz_smooth = accz_smooth + (dT / (fcAcc + dT)) * (accel_ned[Z] - accz_smooth); // low pass filter

    // apply Deadband to reduce integration drift and vibration influence and
    // sum up Values for later integration to get velocity and distance
    accSum[X] += applyDeadband(lrintf(accel_ned[X]), CONFIG_ACCXY_DEADBAND);
    accSum[Y] += applyDeadband(lrintf(accel_ned[Y]), CONFIG_ACCXY_DEADBAND);
    accSum[Z] += applyDeadband(lrintf(accz_smooth), CONFIG_ACCZ_DEADBAND);

    accTimeSum += deltaT;
}



} // extern "C"
