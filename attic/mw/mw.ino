#include "mw.h"


#include <math.h>

#define ROL_LO (1 << (2 * ROLL))
#define ROL_CE (3 << (2 * ROLL))
#define ROL_HI (2 << (2 * ROLL))
#define PIT_LO (1 << (2 * PITCH))
#define PIT_CE (3 << (2 * PITCH))
#define PIT_HI (2 << (2 * PITCH))
#define YAW_LO (1 << (2 * YAW))
#define YAW_CE (3 << (2 * YAW))
#define YAW_HI (2 << (2 * YAW))
#define THR_LO (1 << (2 * THROTTLE))
#define THR_CE (3 << (2 * THROTTLE))
#define THR_HI (2 << (2 * THROTTLE))

#define INBUF_SIZE 128

static uint32_t currentTime;
static uint32_t previousTime;
static int16_t failsafeCnt;
static uint8_t accCalibrated;
static uint8_t dynP8[3], dynI8[3], dynD8[3];

#define PITCH_LOOKUP_LENGTH 7
#define THROTTLE_LOOKUP_LENGTH 12
static int16_t lookupPitchRollRC[PITCH_LOOKUP_LENGTH];   // lookup table for expo & RC rate PITCH+ROLL
static int16_t lookupThrottleRC[THROTTLE_LOOKUP_LENGTH];   // lookup table for expo & mid THROTTLE

static bool haveSmallAngle;

static int16_t  motors[4];
static int16_t  motorsDisarmed[4];

static int16_t angle[2];
static int16_t heading;

static int16_t  rcCommand[4];
static int16_t  rcData[RC_CHANS];

static int16_t  axisPID[3];

static float    anglerad[2];
static float    headingrad;
static uint8_t  armed;
static uint16_t calibratingA;
static uint16_t calibratingG;
static int16_t  gyroADC[3];
static int16_t  accADC[3];

// utilities ======================================================================================================

static void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat)
{
    uint8_t i, r;

    for (r = 0; r < repeat; r++) {
        for (i = 0; i < num; i++) {
            board_led0Toggle();            // switch LEDPIN state
            board_delayMilliseconds(wait);
        }
        board_delayMilliseconds(60);
    }
}

static int constrainer(int amt, int low, int high)
{
    if (amt < low)
        return low;
    else if (amt > high)
        return high;
    else
        return amt;
}

// IMU ============================================================================================================

static uint16_t acc1G;
static int16_t  accSmooth[3];
static int16_t  gyroZero[3];
static float    gyroScale;

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

static void sensorsGetIMU(void)
{
    board_imuRead(accADC, gyroADC);

    int axis = 0;

    static int16_t accZero[3];
    static int32_t a[3];

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

    // range: +/- 8192; +/- 2000 deg/sec

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

static float EstG[3];

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

// rotate acc into Earth frame and calculate acceleration in it
static void acc_calc(uint32_t deltaT)
{
    static int32_t accZoffset = 0;
    static float accz_smooth = 0;
    float dT = 0;
    float rpy[3];
    float accel_ned[3];

    // deltaT is measured in us ticks
    dT = (float)deltaT * 1e-6f;

    // the accel values have to be rotated into the earth frame
    rpy[0] = -(float)anglerad[ROLL];
    rpy[1] = -(float)anglerad[PITCH];
    rpy[2] = -(float)headingrad;

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

// baseflight calculation by Luggi09 originates from arducopter
static float calculateHeading(float vec[3])
{
    float cosineRoll = cosf(anglerad[ROLL]);
    float sineRoll = sinf(anglerad[ROLL]);
    float cosinePitch = cosf(anglerad[PITCH]);
    float sinePitch = sinf(anglerad[PITCH]);
    float Xh = vec[X] * cosinePitch + vec[Y] * sineRoll * sinePitch + vec[Z] * sinePitch * cosineRoll;
    float Yh = vec[Y] * cosineRoll - vec[Z] * sineRoll;
    return atan2f(Yh, Xh); 
}

static void getEstimatedAttitude(void)
{
    int32_t axis;
    int32_t accMag = 0;
    static float EstN[3] = { 1.0f, 0.0f, 0.0f };
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

    rotateV(EstG, deltaGyroAngle);

    // Apply complimentary filter (Gyro drift correction)
    // If accel magnitude >1.15G or <0.85G and ACC vector outside of the limit
    // range => we neutralize the effect of accelerometers in the angle
    // estimation.  To do that, we just skip filter, as EstV already rotated by Gyro
    if (72 < (uint16_t)accMag && (uint16_t)accMag < 133) {
        for (axis = 0; axis < 3; axis++)
            EstG[axis] = (EstG[axis] * (float)CONFIG_GYRO_CMPF_FACTOR + accSmooth[axis]) * INV_GYR_CMPF_FACTOR;
    }

    // Attitude of the estimated vector
    anglerad[ROLL] = atan2f(EstG[Y], EstG[Z]);
    anglerad[PITCH] = atan2f(-EstG[X], sqrtf(EstG[Y] * EstG[Y] + EstG[Z] * EstG[Z]));

    rotateV(EstN, deltaGyroAngle);
    normalizeV(EstN, EstN);
    headingrad = calculateHeading(EstN);

    acc_calc(deltaT); // rotate acc vector into earth frame
}


// MSP ============================================================================================================

typedef enum serialState_t {
    IDLE,
    HEADER_START,
    HEADER_M,
    HEADER_ARROW,
    HEADER_SIZE,
    HEADER_CMD,
} serialState_t;

typedef  struct mspPortState_t {
    uint8_t checksum;
    uint8_t indRX;
    uint8_t inBuf[INBUF_SIZE];
    uint8_t cmdMSP;
    uint8_t offset;
    uint8_t dataSize;
    serialState_t c_state;
} mspPortState_t;


#define MSP_STATUS               101    //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define MSP_RAW_IMU              102    //out message         9 DOF
#define MSP_MOTOR                104    //out message         8 motors
#define MSP_RC                   105    //out message         8 rc chan and more
#define MSP_ATTITUDE             108    //out message         2 angles 1 heading
#define MSP_ALTITUDE             109    //out message         altitude, variometer

#define MSP_BARO_SONAR_RAW       126    // out message

#define MSP_SET_RAW_RC           200    //in message          8 rc chan
#define MSP_SET_MOTOR            214    //in message          PropBalance function


// Additional private MSP for baseflight configurator
#define MSP_REBOOT               68     //in message          reboot settings
#define MSP_BUILDINFO            69     //out message         build date as well as some space for future expansion

// cause reboot after MSP processing complete
static bool pendReboot;
static mspPortState_t portState;
static bool rxMspFrameDone = false;



static void mspFrameReceive(void)
{
    rxMspFrameDone = true;
}

static void serialize8(uint8_t a)
{
    board_serialWrite(a);
    portState.checksum ^= a;
}

static void serialize16(int16_t a)
{
    serialize8(a & 0xFF);
    serialize8((a >> 8) & 0xFF);
}

static void serialize32(uint32_t a)
{
    serialize8(a & 0xFF);
    serialize8((a >> 8) & 0xFF);
    serialize8((a >> 16) & 0xFF);
    serialize8((a >> 24) & 0xFF);
}

static uint8_t read8(void)
{
    return portState.inBuf[portState.indRX++] & 0xff;
}

static uint16_t read16(void)
{
    uint16_t t = read8();
    t += (uint16_t)read8() << 8;
    return t;
}

/*
static uint32_t read32(void)
{
    uint32_t t = read16();
    t += (uint32_t)read16() << 16;
    return t;
}
*/

static void headSerialResponse(uint8_t err, uint8_t s)
{
    serialize8('$');
    serialize8('M');
    serialize8(err ? '!' : '>');
    portState.checksum = 0;               // start calculating a new checksum
    serialize8(s);
    serialize8(portState.cmdMSP);
}

static void headSerialReply(uint8_t s)
{
    headSerialResponse(0, s);
}

static void headSerialError(uint8_t s)
{
    headSerialResponse(1, s);
}

static void tailSerialReply(void)
{
    serialize8(portState.checksum);
}

static void s_struct(uint8_t *cb, uint8_t siz)
{
    headSerialReply(siz);
    while (siz--)
        serialize8(*cb++);
}


static void evaluateCommand(void)
{
    uint32_t i;
    const char *build = __DATE__;

    switch (portState.cmdMSP) {

        case MSP_SET_RAW_RC:
            for (i = 0; i < 8; i++)
                rcData[i] = read16();
            headSerialReply(0);
            mspFrameReceive();
            break;

        case MSP_SET_MOTOR:
            for (i = 0; i < 4; i++)
                motorsDisarmed[i] = read16();
            headSerialReply(0);
            break;

        case MSP_STATUS:
            /*
            headSerialReply(11);
            serialize16(cycleTime);
            serialize16(board_getI2cErrorCounter());
            serialize16(0);
            serialize8(0);
            */
            break;

        case MSP_RAW_IMU:
            break;

        case MSP_MOTOR:
            s_struct((uint8_t *)motors, 16);
            break;

        case MSP_RC:
            headSerialReply(16);
            for (i = 0; i < 8; i++)
                serialize16(rcData[i]);
            break;

        case MSP_ATTITUDE:
            headSerialReply(6);
            for (i = 0; i < 2; i++)
                serialize16(angle[i]);
            serialize16(heading);
            break;

        case MSP_BARO_SONAR_RAW:
            //headSerialReply(8);
            //serialize32(baroPressure);
            //serialize32(sonarDistance);
            break;

        case MSP_ALTITUDE:
            //headSerialReply(6);
            //serialize32(estAlt);
            //serialize16(vario);
            break;

        case MSP_REBOOT:
            headSerialReply(0);
            pendReboot = true;
            break;

        case MSP_BUILDINFO:
            headSerialReply(11 + 4 + 4);
            for (i = 0; i < 11; i++)
                serialize8(build[i]); // MMM DD YYYY as ascii, MMM = Jan/Feb... etc
            serialize32(0); // future exp
            serialize32(0); // future exp
            break;

        default:                   // we do not know how to handle the (valid) message, indicate error MSP $M!
            headSerialError(0);
            break;
    }
    tailSerialReply();
}

static void mspCom(void)
{
    uint8_t c;

    board_checkReboot(pendReboot);

    while (board_serialAvailable()) {

        c = board_serialRead();

        if (portState.c_state == IDLE) {
            portState.c_state = (c == '$') ? HEADER_START : IDLE;
            if (portState.c_state == IDLE && !armed) {
                if (c == '#')
                    ;
                else if (c == CONFIG_REBOOT_CHARACTER) 
                    board_reboot();
            }
        } else if (portState.c_state == HEADER_START) {
            portState.c_state = (c == 'M') ? HEADER_M : IDLE;
        } else if (portState.c_state == HEADER_M) {
            portState.c_state = (c == '<') ? HEADER_ARROW : IDLE;
        } else if (portState.c_state == HEADER_ARROW) {
            if (c > INBUF_SIZE) {       // now we are expecting the payload size
                portState.c_state = IDLE;
                continue;
            }
            portState.dataSize = c;
            portState.offset = 0;
            portState.checksum = 0;
            portState.indRX = 0;
            portState.checksum ^= c;
            portState.c_state = HEADER_SIZE;      // the command is to follow
        } else if (portState.c_state == HEADER_SIZE) {
            portState.cmdMSP = c;
            portState.checksum ^= c;
            portState.c_state = HEADER_CMD;
        } else if (portState.c_state == HEADER_CMD && 
                portState.offset < portState.dataSize) {
            portState.checksum ^= c;
            portState.inBuf[portState.offset++] = c;
        } else if (portState.c_state == HEADER_CMD && 
                portState.offset >= portState.dataSize) {
            if (portState.checksum == c) {        // compare calculated and transferred checksum
                evaluateCommand();      // we got a valid packet, evaluate it
            }
            portState.c_state = IDLE;
        }
    }
}

// Core code ==================================================================================

// Time of automatic disarm when "Don't spin the motors when armed" is enabled.
static uint32_t disarmTime = 0;

static bool check_timed_task(uint32_t usec) 
{

    return (int32_t)(currentTime - usec) >= 0;
}

static void update_timed_task(uint32_t * usec, uint32_t period) 
{
    *usec = currentTime + period;
}

static bool check_and_update_timed_task(uint32_t * usec, uint32_t period) 
{
    bool result = (int32_t)(currentTime - *usec) >= 0;

    if (result)
        update_timed_task(usec, period);

    return result;
}

static void annexCode(void)
{
    static uint32_t calibratedAccTime;
    int32_t tmp, tmp2;
    int32_t axis, prop1, prop2;

    // PITCH & ROLL only dynamic PID adjustemnt,  depending on throttle value
    if (rcData[THROTTLE] < CONFIG_TPA_BREAKPOINT) {
        prop2 = 100;
    } else {
        if (rcData[THROTTLE] < 2000) {
            prop2 = 100 - (uint16_t)CONFIG_DYN_THR_PID * (rcData[THROTTLE] - CONFIG_TPA_BREAKPOINT) / 
                (2000 - CONFIG_TPA_BREAKPOINT);
        } else {
            prop2 = 100 - CONFIG_DYN_THR_PID;
        }
    }

    for (axis = 0; axis < 3; axis++) {
        tmp = min(abs(rcData[axis] - CONFIG_MIDRC), 500);
        if (axis != 2) {        // ROLL & PITCH
            if (CONFIG_DEADBAND) {
                if (tmp > CONFIG_DEADBAND) {
                    tmp -= CONFIG_DEADBAND;
                } else {
                    tmp = 0;
                }
            }

            tmp2 = tmp / 100;
            rcCommand[axis] = lookupPitchRollRC[tmp2] + (tmp - tmp2 * 100) * (lookupPitchRollRC[tmp2 + 1] - 
                    lookupPitchRollRC[tmp2]) / 100;
            prop1 = 100 - (uint16_t)CONFIG_ROLL_PITCH_RATE[axis] * tmp / 500;
            prop1 = (uint16_t)prop1 * prop2 / 100;
        } else {                // YAW
            if (CONFIG_YAW_DEADBAND) {
                if (tmp > CONFIG_YAW_DEADBAND) {
                    tmp -= CONFIG_YAW_DEADBAND;
                } else {
                    tmp = 0;
                }
            }
            rcCommand[axis] = tmp * -CONFIG_YAW_CONTROL_DIRECTION;
            prop1 = 100 - (uint16_t)CONFIG_YAW_RATE * abs(tmp) / 500;
        }
        dynP8[axis] = (uint16_t)CONFIG_AXIS_P[axis] * prop1 / 100;
        dynI8[axis] = (uint16_t)CONFIG_AXIS_I[axis] * prop1 / 100;
        dynD8[axis] = (uint16_t)CONFIG_AXIS_D[axis] * prop1 / 100;
        if (rcData[axis] < CONFIG_MIDRC)
            rcCommand[axis] = -rcCommand[axis];
    }

    tmp = constrainer(rcData[THROTTLE], CONFIG_MINCHECK, 2000);
    tmp = (uint32_t)(tmp - CONFIG_MINCHECK) * 1000 / (2000 - CONFIG_MINCHECK);       // [MINCHECK;2000] -> [0;1000]
    tmp2 = tmp / 100;
    rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp - tmp2 * 100) * (lookupThrottleRC[tmp2 + 1] - 
            lookupThrottleRC[tmp2]) / 100;    // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]

    if (calibratingA > 0 || calibratingG > 0) {      // Calibration phasis
        board_led0Toggle();
    } else {
        if (accCalibrated)
            board_led0Off();
        if (armed)
            board_led0On();
    }

    if (check_timed_task(calibratedAccTime)) {
        if (!haveSmallAngle) {
            accCalibrated = 0; // the multi uses ACC and is not calibrated or is too much inclinated
            board_led0Toggle();
            update_timed_task(&calibratedAccTime, CONFIG_CALIBRATE_ACCTIME_USEC);
        } else {
            accCalibrated = 1;
        }
    }

    mspCom();
}

static void computeRC(void)
{
    uint16_t capture;
    int i, chan;

    static int16_t rcDataAverage[8][4];
    static int rcAverageIndex = 0;

    for (chan = 0; chan < 8; chan++) {
    
        // get RC PWM
        capture = board_pwmRead(CONFIG_RCMAP[chan]);

        // XXX default to CONFIG_MIDRC if out-of-bounds
        //if (capture < PULSE_MIN || capture > PULSE_MAX)
        //   capture =  CONFIG_MIDRC;

        rcDataAverage[chan][rcAverageIndex % 4] = capture;

        // clear this since we're not accessing it elsewhere. saves a temp var
        rcData[chan] = 0;
        for (i = 0; i < 4; i++)
            rcData[chan] += rcDataAverage[chan][i];
        rcData[chan] /= 4;
    }

    rcAverageIndex++;
}

static void mwArm(void)
{
    if (calibratingG == 0 && accCalibrated) {
        if (!armed) {         // arm now!
            armed = 1;
        }
    } else if (!armed) {
        blinkLED(2, 255, 1);
    }
}

static void mwDisarm(void)
{
    if (armed) {
        armed = 0;
        // Reset disarm time so that it works next time we arm the board.
        if (disarmTime != 0)
            disarmTime = 0;
    }
}

static int32_t errorGyroI[3] = { 0, 0, 0 };
static int32_t errorAngleI[2] = { 0, 0 };

static void pidMultiWii(void)
{
    int axis, prop;
    int32_t error, errorAngle;
    int32_t PTerm, ITerm, PTermACC = 0, ITermACC = 0, PTermGYRO = 0, ITermGYRO = 0, DTerm;
    static int16_t lastGyro[3] = { 0, 0, 0 };
    static int32_t delta1[3], delta2[3];
    int32_t deltaSum;
    int32_t delta;

    // **** PITCH & ROLL & YAW PID ****
    prop = max(abs(rcCommand[PITCH]), abs(rcCommand[ROLL])); // range [0;500]
    for (axis = 0; axis < 3; axis++) {
        if ((CONFIG_HORIZON_MODE) && axis < 2) { // MODE relying on ACC
            // 50 degrees max inclination
            errorAngle = constrainer(2 * rcCommand[axis], -((int)CONFIG_MAX_ANGLE_INCLINATION), 
                    + CONFIG_MAX_ANGLE_INCLINATION) - angle[axis] + CONFIG_ANGLE_TRIM[axis];
            PTermACC = errorAngle * CONFIG_LEVEL_P / 100; 
            // 32 bits is needed for calculation: errorAngle*CONFIG_LEVEL_P could exceed 32768   
            // 16 bits is ok for result
            PTermACC = constrainer(PTermACC, -CONFIG_LEVEL_D * 5, + CONFIG_LEVEL_D * 5);

            errorAngleI[axis] = constrainer(errorAngleI[axis] + errorAngle, -10000, +10000); // WindUp
            ITermACC = (errorAngleI[axis] * CONFIG_LEVEL_I) >> 12;
        }
        if (CONFIG_HORIZON_MODE || axis == 2) { // MODE relying on GYRO or YAW axis
            error = (int32_t)rcCommand[axis] * 10 * 8 / CONFIG_AXIS_P[axis];
            error -= gyroADC[axis];

            PTermGYRO = rcCommand[axis];

            errorGyroI[axis] = constrainer(errorGyroI[axis] + error, -16000, +16000); // WindUp
            if ((abs(gyroADC[axis]) > 640) || ((axis == YAW) && (abs(rcCommand[axis]) > 100)))
                errorGyroI[axis] = 0;
            ITermGYRO = (errorGyroI[axis] / 125 * CONFIG_AXIS_I[axis]) >> 6;
        }
        if (CONFIG_HORIZON_MODE && axis < 2) {
            PTerm = (PTermACC * (500 - prop) + PTermGYRO * prop) / 500;
            ITerm = (ITermACC * (500 - prop) + ITermGYRO * prop) / 500;
        } else {
            PTerm = PTermGYRO;
            ITerm = ITermGYRO;
        }

        PTerm -= (int32_t)gyroADC[axis] * dynP8[axis] / 10 / 8; // 32 bits is needed for calculation
        delta = gyroADC[axis] - lastGyro[axis];
        lastGyro[axis] = gyroADC[axis];
        deltaSum = delta1[axis] + delta2[axis] + delta;
        delta2[axis] = delta1[axis];
        delta1[axis] = delta;
        DTerm = (deltaSum * dynD8[axis]) / 32;
        axisPID[axis] = PTerm + ITerm - DTerm;
    }
}


// Mixer =========================================================================================================


// Custom mixer data per motor
typedef struct motorMixer_t {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} motorMixer_t;

static const motorMixer_t mixerQuadX[] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
};

static void mixerInit(void)
{
    int i;

    // set disarmed motor values
    for (i = 0; i < 4; i++)
        motorsDisarmed[i] = CONFIG_MINCOMMAND;
}

static void mixerWriteMotors(void)
{
    int16_t maxMotor;
    uint32_t i;

    // prevent "yaw jump" during yaw correction
    axisPID[YAW] = constrainer(axisPID[YAW], -100 - abs(rcCommand[YAW]), +100 + abs(rcCommand[YAW]));

    for (i = 0; i < 4; i++)
        motors[i] = rcCommand[THROTTLE] * mixerQuadX[i].throttle + axisPID[PITCH] * mixerQuadX[i].pitch + 
            axisPID[ROLL] * mixerQuadX[i].roll + -CONFIG_YAW_DIRECTION * axisPID[YAW] * mixerQuadX[i].yaw;

    maxMotor = motors[0];
    for (i = 1; i < 4; i++)
        if (motors[i] > maxMotor)
            maxMotor = motors[i];
    for (i = 0; i < 4; i++) {
        if (maxMotor > CONFIG_MAXTHROTTLE)     
            // this is a way to still have good gyro corrections if at least one motor reaches its max.
            motors[i] -= maxMotor - CONFIG_MAXTHROTTLE;

        motors[i] = constrainer(motors[i], CONFIG_MINTHROTTLE, CONFIG_MAXTHROTTLE);
        if ((rcData[THROTTLE]) < CONFIG_MINCHECK) {
            motors[i] = CONFIG_MINTHROTTLE;
        } 
        if (!armed) {
            motors[i] = motorsDisarmed[i];
        }
    }

    for (i = 0; i < 4; i++)
        board_writeMotor(i, motors[i]);
}


// =================================================================================================================

void setup(void)
{
    uint8_t i;

    board_init();

    // sleep for 100ms
    board_delayMilliseconds(100);

    for (i = 0; i < PITCH_LOOKUP_LENGTH; i++)
        lookupPitchRollRC[i] = (2500 + CONFIG_RC_EXPO_8 * (i * i - 25)) * i * (int32_t)CONFIG_RC_RATE_8 / 2500;

    for (i = 0; i < THROTTLE_LOOKUP_LENGTH; i++) {
        int16_t tmp = 10 * i - CONFIG_THR_MID_8;
        uint8_t y = 1;
        if (tmp > 0)
            y = 100 - CONFIG_THR_MID_8;
        if (tmp < 0)
            y = CONFIG_THR_MID_8;
        lookupThrottleRC[i] = 10 * CONFIG_THR_MID_8 + tmp * (100 - CONFIG_THR_EXPO_8 + 
                (int32_t)CONFIG_THR_EXPO_8 * (tmp * tmp) / (y * y)) / 10;
        lookupThrottleRC[i] = CONFIG_MINTHROTTLE + (int32_t)(CONFIG_MAXTHROTTLE - CONFIG_MINTHROTTLE) * 
            lookupThrottleRC[i] / 1000; // [MINTHROTTLE;MAXTHROTTLE]
    }

    board_led1On();
    board_led0Off();
    for (i = 0; i < 10; i++) {
        board_led1Toggle();
        board_led0Toggle();
        board_delayMilliseconds(50);
    }
    board_led1Off();
    board_led0Off();

    board_imuInit(&acc1G, &gyroScale);

    mixerInit(); 

    // configure PWM/CPPM read function and max number of channels
    // these, if enabled
    for (i = 0; i < RC_CHANS; i++)
        rcData[i] = 1502;

    previousTime = board_getMicros();
    calibratingG = CONFIG_CALIBRATING_GYRO_CYCLES;
    // 10 seconds init_delay + 200 * 25 ms = 15 seconds before ground pressure settles

    // trigger accelerometer calibration requirement
    haveSmallAngle = true;
 }

void loop(void)
{
    static uint8_t rcDelayCommand;      // this indicates the number of time (multiple of RC measurement at 50Hz) 
    // the sticks must be maintained to run or switch off motors
    static uint8_t rcSticks;            // this hold sticks position for command combos
    uint8_t stTmp = 0;
    int i;
    static uint32_t rcTime = 0;
    static uint32_t loopTime;
    uint16_t auxState = 0;
    bool isThrottleLow = false;

    if (check_and_update_timed_task(&rcTime, CONFIG_RC_LOOPTIME_USEC)) {

        computeRC();

        // ------------------ STICKS COMMAND HANDLER --------------------
        // checking sticks positions
        for (i = 0; i < 4; i++) {
            stTmp >>= 2;
            if (rcData[i] > CONFIG_MINCHECK)
                stTmp |= 0x80;  // check for MIN
            if (rcData[i] < CONFIG_MAXCHECK)
                stTmp |= 0x40;  // check for MAX
        }
        if (stTmp == rcSticks) {
            if (rcDelayCommand < 250)
                rcDelayCommand++;
        } else
            rcDelayCommand = 0;
        rcSticks = stTmp;

        // perform actions
        if ((rcData[THROTTLE] < CONFIG_MINCHECK))
            isThrottleLow = true;
        if (isThrottleLow) {
            errorGyroI[ROLL] = 0;
            errorGyroI[PITCH] = 0;
            errorGyroI[YAW] = 0;
            errorAngleI[ROLL] = 0;
            errorAngleI[PITCH] = 0;
        }

        if (rcDelayCommand == 20) {
            if (armed) {      // actions during armed
                // Disarm on throttle down + yaw
                if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_CE)
                    mwDisarm();
            } else {            // actions during not armed

                // GYRO calibration
                if (rcSticks == THR_LO + YAW_LO + PIT_LO + ROL_CE) {
                    calibratingG = CONFIG_CALIBRATING_GYRO_CYCLES;
                } 

                // Arm via YAW
                if ((rcSticks == THR_LO + YAW_HI + PIT_CE + ROL_CE))
                    mwArm();

                // Calibrating Acc
                else if (rcSticks == THR_HI + YAW_LO + PIT_LO + ROL_CE)
                    calibratingA = CONFIG_CALIBRATING_ACC_CYCLES;

                i = 0;
                if (i) {
                    rcDelayCommand = 0; // allow autorepetition
                }
            }
        }

        // Check AUX switches
        for (i = 0; i < 4; i++)
            auxState |= (rcData[AUX1 + i] < 1300) << (3 * i) | (1300 < rcData[AUX1 + i] && rcData[AUX1 + i] < 1700) 
                << (3 * i + 1) | (rcData[AUX1 + i] > 1700) << (3 * i + 2);

        // note: if FAILSAFE is disable, failsafeCnt > 5 * FAILSAVE_DELAY is always false
        if (failsafeCnt > 5 * CONFIG_FAILSAFE_DELAY) {
            // bumpless transfer to Level mode
            errorAngleI[ROLL] = 0;
            errorAngleI[PITCH] = 0;
        } 

    } else {                        // not in rc loop
        static int taskOrder = 0;   // never call all functions in the same loop, to avoid high delay spikes
        switch (taskOrder) {
            case 0:
                taskOrder++;
                //sensorsGetBaro();
            case 1:
                taskOrder++;
                //sensorsGetSonar();
            case 2:
                taskOrder++;
            case 3:
                taskOrder++;
            case 4:
                taskOrder = 0;
                break;
        }
    }
    
    currentTime = board_getMicros();

    if (check_and_update_timed_task(&loopTime, CONFIG_IMU_LOOPTIME_USEC)) {

        sensorsGetIMU();
        getEstimatedAttitude();

        angle[ROLL] = lrintf(anglerad[ROLL] * (1800.0f / M_PI));
        angle[PITCH] = lrintf(anglerad[PITCH] * (1800.0f / M_PI));

        heading = lrintf(headingrad * 1800.0f / M_PI + CONFIG_MAGNETIC_DECLINATION) / 10.0f;

        if (heading < 0)
            heading += 360;

        haveSmallAngle = abs(angle[0]) < CONFIG_SMALL_ANGLE && abs(angle[1]) < CONFIG_SMALL_ANGLE;

        // Measure loop rate just afer reading the sensors
        currentTime = board_getMicros();
        previousTime = currentTime;

        // non IMU critical, temeperatur, serialcom
        annexCode();

        pidMultiWii();

        mixerWriteMotors();
    }
}
