#define EXTERN 

#include "mw.h"
#include "config.h"
#include "board.h"

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

static uint32_t currentTime;
static uint32_t previousTime;
static int16_t failsafeCnt;
static uint8_t accCalibrated;
static uint8_t dynP8[3], dynI8[3], dynD8[3];

#define PITCH_LOOKUP_LENGTH 7
#define THROTTLE_LOOKUP_LENGTH 12
static int16_t lookupPitchRollRC[PITCH_LOOKUP_LENGTH];   // lookup table for expo & RC rate PITCH+ROLL
static int16_t lookupThrottleRC[THROTTLE_LOOKUP_LENGTH];   // lookup table for expo & mid THROTTLE

// Time of automatic disarm when "Don't spin the motors when armed" is enabled.
static uint32_t disarmTime = 0;

static void ledToggle(void)
{
    static bool state;

    if (state)
        board_ledOff();
    else
        board_ledOn();

    state = !state;
}

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
        ledToggle();
    } else {
        if (accCalibrated)
            board_ledOff();
        if (armed)
            board_ledOn();
    }

    if (check_timed_task(calibratedAccTime)) {
        if (!useSmallAngle) {
            accCalibrated = 0; // the multi uses ACC and is not calibrated or is too much inclinated
            ledToggle();
            update_timed_task(&calibratedAccTime, CONFIG_CALIBRATE_ACCTIME_USEC);
            //calibratedAccTime = currentTime + CONFIG_CALIBRATE_ACCTIME_USEC;
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

    sensorsInit();

    board_ledOff();
    for (i = 0; i < 10; i++) {
        ledToggle();
        board_delayMilliseconds(50);
    }
    board_ledOff();

    stateInit(); 
    mixerInit(); 

    // configure PWM/CPPM read function and max number of channels
    // these, if enabled
    for (i = 0; i < RC_CHANS; i++)
        rcData[i] = 1502;

    previousTime = board_getMicros();
    calibratingG = CONFIG_CALIBRATING_GYRO_CYCLES;
    // 10 seconds init_delay + 200 * 25 ms = 15 seconds before ground pressure settles

    // trigger accelerometer calibration requirement
    useSmallAngle = true;
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

    //static uint16_t targetAGLcm;
    //static uint8_t  altHold;


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
                i = 0;
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
                sensorsGetBaro();
            case 1:
                taskOrder++;
                sensorsGetSonar();
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

        stateComputeAngles();

        // Measure loop rate just afer reading the sensors
        currentTime = board_getMicros();
        cycleTime = (int32_t)(currentTime - previousTime);
        previousTime = currentTime;

        // non IMU critical, temeperatur, serialcom
        annexCode();

        pidMultiWii();

        mixerWriteMotors();
    }
}

void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat)
{
    uint8_t i, r;

    for (r = 0; r < repeat; r++) {
        for (i = 0; i < num; i++) {
            ledToggle();            // switch LEDPIN state
            board_delayMilliseconds(wait);
        }
        board_delayMilliseconds(60);
    }
}

int constrainer(int amt, int low, int high)
{
    if (amt < low)
        return low;
    else if (amt > high)
        return high;
    else
        return amt;
}
