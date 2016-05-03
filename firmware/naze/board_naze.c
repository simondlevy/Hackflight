#include <breezystm32.h>
#include <math.h>
#include "config.h"
#include "mw.h"

#define USE_CPPM                             1
#define PWM_FILTER                           0     /* 0 or 1 */
#define FAST_PWM                             0     /* 0 or 1 */
#define MOTOR_PWM_RATE                       400
#define PWM_IDLE_PULSE                       1000  /* standard PWM in usec for brushless ESC */


static uint16_t acc1G;
static int16_t  accADC[3];
static int16_t  accSmooth[3];
static int16_t  gyroZero[3];
static float    gyroScale;

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

static void sensorsGetAccel(void)
{
    mpu6050_read_accel(accADC);

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

    mpu6050_read_gyro(gyroADC);

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
    rpy[2] = -(float)heading * (M_PI / 180.0f);

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

void board_imuInit(void)
{
    mpu6050_init(false, &acc1G, &gyroScale);
}

void board_imuComputeAngles(void)
{
    sensorsGetGyro();
    sensorsGetAccel();
    getEstimatedAttitude();
}

extern serialPort_t * Serial1;

void board_init(void)
{
    i2cInit(I2CDEV_2);
    pwmInit(USE_CPPM, PWM_FILTER, FAST_PWM, MOTOR_PWM_RATE, PWM_IDLE_PULSE);
}

bool board_baroInit(void)
{
    return ms5611_init();
}

int32_t board_baroReadPressure(void)
{
    return ms5611_read_pressure();
}

void board_checkReboot(bool pendReboot)
{
    if (pendReboot)
        systemReset(false); // noreturn
}

void board_delayMilliseconds(uint32_t msec)
{
    delay(msec);
}

uint16_t board_getI2cErrorCounter(void)
{
    return i2cGetErrorCounter();
}

uint32_t board_getMicros()
{
    return micros();
}

void board_ledOff(void)
{
    LED0_OFF;
}

void board_ledOn(void)
{
    LED0_ON;
}


uint16_t board_pwmRead(uint8_t chan)
{
    return pwmRead(chan);
}

void board_reboot(void)
{
    systemReset(true);      // reboot to bootloader
}

uint8_t board_serialAvailable(void)
{
    return serialTotalBytesWaiting(Serial1);
}

uint8_t board_serialRead(void)
{
    return serialRead(Serial1);
}

void board_serialWrite(uint8_t c)
{
    serialWrite(Serial1, c);
}

void board_writeMotor(uint8_t index, uint16_t value)
{
    pwmWriteMotor(index, value);
}

bool board_sonarInit(void)
{
    return mb1242_init();
}

int32_t board_sonarReadDistance(void)
{
    return mb1242_poll();
}



