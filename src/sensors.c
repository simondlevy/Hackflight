#include "mw.h"
#include "config.h"
#include "board.h"

#include <math.h>

#define BARO_TAB_SIZE_MAX 48

// Default orientation
static sensor_align_e gyroAlign = CW0_DEG;
static sensor_align_e accAlign = CW0_DEG;

static bool baroAvailable;
static bool sonarAvailable;

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


// ==============================================================================================================

void sensorsInit(void)
{
    board_i2cInit();

    board_imuInit(CONFIG_GYRO_LPF, &acc1G, &gyroScale);

    baroAvailable = board_baroInit();
    sonarAvailable = board_sonarInit();

    // Default to OOB value to indicate sensor unavailable
    baroPressure = -1;
    sonarDistance = -1;
}

void sensorsGetAccel(void)
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


void sensorsGetGyro(void)
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

void sensorsGetBaro(void)
{
    if (baroAvailable) 
        baroPressure = board_baroReadPressure();
}

void sensorsGetSonar(void)
{
    if (sonarAvailable) 
        sonarDistance  = board_sonarReadDistance();
}
 
