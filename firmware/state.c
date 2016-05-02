#include "mw.h"
#include "config.h"
#include "board.h"

#include <math.h>

#define RAD    (M_PI / 180.0f)


// Default orientation
static sensor_align_e gyroAlign = CW0_DEG;
static sensor_align_e accAlign = CW0_DEG;

typedef struct stdev_t {
    float m_oldM, m_newM, m_oldS, m_newS;
    int m_n;
} stdev_t;

static void devClear(stdev_t *dev)
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

static void alignSensors(int16_t *src, int16_t *dest, uint8_t rotation)
{
    switch (rotation) {
        case CW0_DEG:
            dest[X] = src[X];
            dest[Y] = src[Y];
            dest[Z] = src[Z];
            break;
        case CW90_DEG:
            dest[X] = src[Y];
            dest[Y] = -src[X];
            dest[Z] = src[Z];
            break;
        case CW180_DEG:
            dest[X] = -src[X];
            dest[Y] = -src[Y];
            dest[Z] = src[Z];
            break;
        case CW270_DEG:
            dest[X] = -src[Y];
            dest[Y] = src[X];
            dest[Z] = src[Z];
            break;
        case CW0_DEG_FLIP:
            dest[X] = -src[X];
            dest[Y] = src[Y];
            dest[Z] = -src[Z];
            break;
        case CW90_DEG_FLIP:
            dest[X] = src[Y];
            dest[Y] = src[X];
            dest[Z] = -src[Z];
            break;
        case CW180_DEG_FLIP:
            dest[X] = src[X];
            dest[Y] = -src[Y];
            dest[Z] = -src[Z];
            break;
        case CW270_DEG_FLIP:
            dest[X] = -src[Y];
            dest[Y] = -src[X];
            dest[Z] = -src[Z];
            break;
        default:
            break;
    }
}

static void sensorsGetAccel(void)
{
    static int16_t data[3];
    board_imuReadAccel(data);
    alignSensors(data, accADC, accAlign);

    static int16_t accZero[3];
    static int32_t a[3];
    int axis;

    if (calibratingA > 0) {
        for (axis = 0; axis < 3; axis++) {
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
            accZero[YAW] = (a[YAW] + (CONFIG_CALIBRATING_ACC_CYCLES / 2)) / CONFIG_CALIBRATING_ACC_CYCLES - acc1G;
        }
        calibratingA--;
    }

    accADC[ROLL] -= accZero[ROLL];
    accADC[PITCH] -= accZero[PITCH];
    accADC[YAW] -= accZero[YAW];
}


static void sensorsGetGyro(void)
{
    // range: +/- 8192; +/- 2000 deg/sec

    static int16_t data[3];
    board_imuReadGyro(data);
    alignSensors(data, gyroADC, gyroAlign);

    int axis;
    static int32_t g[3];
    static stdev_t var[3];

    if (calibratingG > 0) {
        for (axis = 0; axis < 3; axis++) {
            // Reset g[axis] at start of calibration
            if (calibratingG == CONFIG_CALIBRATING_GYRO_CYCLES) {
                g[axis] = 0;
                devClear(&var[axis]);
            }
            // Sum up 1000 readings
            g[axis] += gyroADC[axis];
            devPush(&var[axis], gyroADC[axis]);
            // Clear global variables for next reading
            gyroADC[axis] = 0;
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
                blinkLED(10, 15, 1);
            }
        }
        calibratingG--;
    }
    for (axis = 0; axis < 3; axis++)
        gyroADC[axis] -= gyroZero[axis];
}

static int32_t  accSum[3];
static uint32_t accTimeSum;        // keep track for integration of acc
static float    fcAcc = 0.5f / (M_PI * CONFIG_ACCZ_LPF_CUTOFF); // calculate RC time constant used in the accZ lpf

// **************************************************
// Simplified IMU based on "Complementary Filter"
// Inspired by http://starlino.com/imu_guide.html
//
// adapted by ziss_dm : http://www.multiwii.com/forum/viewtopic.php?f=8&t=198
//
// The following ideas was used in this project:
// 1) Rotation matrix: http://en.wikipedia.org/wiki/Rotation_matrix
//
// Currently Magnetometer uses separate CF which is used only
// for heading approximation.
//
// **************************************************

#define INV_GYR_CMPF_FACTOR   (1.0f / ((float)CONFIG_GYRO_CMPF_FACTOR + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / ((float)CONFIG_GYRO_CMPFM_FACTOR + 1.0f))

typedef struct fp_vector {
    float X;
    float Y;
    float Z;
} t_fp_vector_def;

typedef union {
    float A[3];
    t_fp_vector_def V;
} t_fp_vector;

t_fp_vector EstG;

// Normalize a vector
static void normalizeV(struct fp_vector *src, struct fp_vector *dest)
{
    float length;

    length = sqrtf(src->X * src->X + src->Y * src->Y + src->Z * src->Z);
    if (length != 0) {
        dest->X = src->X / length;
        dest->Y = src->Y / length;
        dest->Z = src->Z / length;
    }
}

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
static void rotateV(struct fp_vector *v, float *delta)
{
    struct fp_vector v_tmp = *v;

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

    v->X = v_tmp.X * mat[0][0] + v_tmp.Y * mat[1][0] + v_tmp.Z * mat[2][0];
    v->Y = v_tmp.X * mat[0][1] + v_tmp.Y * mat[1][1] + v_tmp.Z * mat[2][1];
    v->Z = v_tmp.X * mat[0][2] + v_tmp.Y * mat[1][2] + v_tmp.Z * mat[2][2];
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

// rotate acc into Earth frame and calculate acceleration in it
static void acc_calc(uint32_t deltaT)
{
    static int32_t accZoffset = 0;
    static float accz_smooth = 0;
    float dT = 0;
    float rpy[3];
    t_fp_vector accel_ned;

    // deltaT is measured in us ticks
    dT = (float)deltaT * 1e-6f;

    // the accel values have to be rotated into the earth frame
    rpy[0] = -(float)anglerad[ROLL];
    rpy[1] = -(float)anglerad[PITCH];
    rpy[2] = -(float)heading * RAD;

    accel_ned.V.X = accSmooth[0];
    accel_ned.V.Y = accSmooth[1];
    accel_ned.V.Z = accSmooth[2];

    rotateV(&accel_ned.V, rpy);

    if (CONFIG_ACC_UNARMEDCAL == 1) {
        if (!armed) {
            accZoffset -= accZoffset / 64;
            accZoffset += accel_ned.V.Z;
        }
        accel_ned.V.Z -= accZoffset / 64;  // compensate for gravitation on z-axis
    } else
        accel_ned.V.Z -= acc1G;

    accz_smooth = accz_smooth + (dT / (fcAcc + dT)) * (accel_ned.V.Z - accz_smooth); // low pass filter

    // apply Deadband to reduce integration drift and vibration influence and
    // sum up Values for later integration to get velocity and distance
    accSum[X] += applyDeadband(lrintf(accel_ned.V.X), CONFIG_ACCXY_DEADBAND);
    accSum[Y] += applyDeadband(lrintf(accel_ned.V.Y), CONFIG_ACCXY_DEADBAND);
    accSum[Z] += applyDeadband(lrintf(accz_smooth), CONFIG_ACCZ_DEADBAND);

    accTimeSum += deltaT;
}

// baseflight calculation by Luggi09 originates from arducopter
static int16_t calculateHeading(t_fp_vector *vec)
{
    int16_t head;

    float cosineRoll = cosf(anglerad[ROLL]);
    float sineRoll = sinf(anglerad[ROLL]);
    float cosinePitch = cosf(anglerad[PITCH]);
    float sinePitch = sinf(anglerad[PITCH]);
    float Xh = vec->A[X] * cosinePitch + vec->A[Y] * sineRoll * sinePitch + vec->A[Z] * sinePitch * cosineRoll;
    float Yh = vec->A[Y] * cosineRoll - vec->A[Z] * sineRoll;
    float hd = (atan2f(Yh, Xh) * 1800.0f / M_PI + CONFIG_MAGNETIC_DECLINATION) / 10.0f;
    head = lrintf(hd);
    if (head < 0)
        head += 360;

    return head;
}

static void getEstimatedAttitude(void)
{
    int32_t axis;
    int32_t accMag = 0;
    static t_fp_vector EstN = { .A = { 1.0f, 0.0f, 0.0f } };
    static float accLPF[3];
    static uint32_t previousT;
    uint32_t currentT = board_getMicros();
    uint32_t deltaT;
    float deltaGyroAngle[3];
    deltaT = currentT - previousT;
    float scale = deltaT * gyroScale;
    previousT = currentT;

    // Initialization
    for (axis = 0; axis < 3; axis++) {
        deltaGyroAngle[axis] = gyroADC[axis] * scale;
        if (CONFIG_ACC_LPF_FACTOR > 0) {
            accLPF[axis] = accLPF[axis] * (1.0f - (1.0f / CONFIG_ACC_LPF_FACTOR)) + accADC[axis] * 
                (1.0f / CONFIG_ACC_LPF_FACTOR);
            accSmooth[axis] = accLPF[axis];
        } else {
            accSmooth[axis] = accADC[axis];
        }
        accMag += (int32_t)accSmooth[axis] * accSmooth[axis];
    }
    accMag = accMag * 100 / ((int32_t)acc1G * acc1G);

    rotateV(&EstG.V, deltaGyroAngle);

    // Apply complimentary filter (Gyro drift correction)
    // If accel magnitude >1.15G or <0.85G and ACC vector outside of the limit
    // range => we neutralize the effect of accelerometers in the angle
    // estimation.  To do that, we just skip filter, as EstV already rotated by Gyro
    if (72 < (uint16_t)accMag && (uint16_t)accMag < 133) {
        for (axis = 0; axis < 3; axis++)
            EstG.A[axis] = (EstG.A[axis] * (float)CONFIG_GYRO_CMPF_FACTOR + accSmooth[axis]) * INV_GYR_CMPF_FACTOR;
    }

    // Attitude of the estimated vector
    anglerad[ROLL] = atan2f(EstG.V.Y, EstG.V.Z);
    anglerad[PITCH] = atan2f(-EstG.V.X, sqrtf(EstG.V.Y * EstG.V.Y + EstG.V.Z * EstG.V.Z));

    rotateV(&EstN.V, deltaGyroAngle);
    normalizeV(&EstN.V, &EstN.V);
    heading = calculateHeading(&EstN);

    acc_calc(deltaT); // rotate acc vector into earth frame
}

// =================================================================================================================

void stateComputeAngles(void)
{
    sensorsGetGyro();
    sensorsGetAccel();
    getEstimatedAttitude();
}


