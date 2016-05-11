extern "C" {

#include "mw.hpp"
#include "imu.hpp"
#include "mixer.hpp"
#include "msp.hpp"

#ifndef PRINTF
#define PRINTF printf
#endif

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

#define PITCH_LOOKUP_LENGTH 7
#define THROTTLE_LOOKUP_LENGTH 12

static int16_t  angle[3];
static int16_t  axisPID[3];
static uint16_t calibratingA;
static uint16_t calibratingG;
static uint32_t currentTime;
static int16_t  lookupPitchRollRC[PITCH_LOOKUP_LENGTH];   // lookup table for expo & RC rate PITCH+ROLL
static int16_t  lookupThrottleRC[THROTTLE_LOOKUP_LENGTH];   // lookup table for expo & mid THROTTLE
static bool     haveSmallAngle;
static uint32_t previousTime;
static int16_t  rcData[RC_CHANS];

// utilities ======================================================================================================

static void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat)
{
    for (uint8_t r = 0; r < repeat; r++) {
        for (uint8_t i = 0; i < num; i++) {
            board_led0Toggle();            // switch LEDPIN state
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

IMU   imu;
Mixer mixer;
MSP   msp;

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

static void computeRCExpo(int16_t rcCommand[4])
{
    int32_t tmp, tmp2;

    for (uint8_t axis = 0; axis < 3; axis++) {

        tmp = min(abs(rcData[axis] - CONFIG_MIDRC), 500);

        if (axis != 2) {        // ROLL & PITCH
            tmp2 = tmp / 100;
            rcCommand[axis] = lookupPitchRollRC[tmp2] + (tmp - tmp2 * 100) * (lookupPitchRollRC[tmp2 + 1] - lookupPitchRollRC[tmp2]) / 100;
        } else {                // YAW
            rcCommand[axis] = tmp * -CONFIG_YAW_CONTROL_DIRECTION;
        }

        if (rcData[axis] < CONFIG_MIDRC)
            rcCommand[axis] = -rcCommand[axis];
    }

    tmp = constrainer(rcData[THROTTLE], CONFIG_MINCHECK, 2000);
    tmp = (uint32_t)(tmp - CONFIG_MINCHECK) * 1000 / (2000 - CONFIG_MINCHECK);       // [MINCHECK;2000] -> [0;1000]
    tmp2 = tmp / 100;
    rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp - tmp2 * 100) * (lookupThrottleRC[tmp2 + 1] - 
            lookupThrottleRC[tmp2]) / 100;    // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]

} // computeRCExpo

static void computeRC(void)
{

    static int16_t rcDataAverage[8][4];
    static int rcAverageIndex = 0;

    for (uint8_t chan = 0; chan < 8; chan++) {
    
        // get RC PWM
        rcDataAverage[chan][rcAverageIndex % 4] = board_pwmRead(CONFIG_RCMAP[chan]);

        rcData[chan] = 0;

        for (uint8_t i = 0; i < 4; i++)
            rcData[chan] += rcDataAverage[chan][i];
        rcData[chan] /= 4;
    }

    rcAverageIndex++;
}

static int32_t errorGyroI[3] = { 0, 0, 0 };
static int32_t errorAngleI[2] = { 0, 0 };

static void pidMultiWii(int16_t rcCommand[4])
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
            error -= imu.gyroADC[axis];

            PTermGYRO = rcCommand[axis];

            errorGyroI[axis] = constrainer(errorGyroI[axis] + error, -16000, +16000); // WindUp
            if ((abs(imu.gyroADC[axis]) > 640) || ((axis == YAW) && (abs(rcCommand[axis]) > 100)))
                errorGyroI[axis] = 0;
            ITermGYRO = (errorGyroI[axis] / 125 * CONFIG_AXIS_I[axis]) >> 6;
        }

        int32_t PTerm = PTermGYRO;
        int32_t ITerm = ITermGYRO;

        if (CONFIG_HORIZON_MODE && axis < 2) {
            PTerm = (PTermACC * (500 - prop) + PTermGYRO * prop) / 500;
            ITerm = (ITermACC * (500 - prop) + ITermGYRO * prop) / 500;
        } 

        PTerm -= (int32_t)imu.gyroADC[axis] * CONFIG_AXIS_P[axis] / 10 / 8; // 32 bits is needed for calculation
        int32_t delta = imu.gyroADC[axis] - lastGyro[axis];
        lastGyro[axis] = imu.gyroADC[axis];
        int32_t deltaSum = delta1[axis] + delta2[axis] + delta;
        delta2[axis] = delta1[axis];
        delta1[axis] = delta;
        int32_t DTerm = (deltaSum * CONFIG_AXIS_D[axis]) / 32;
        axisPID[axis] = PTerm + ITerm - DTerm;
    }
}


// =================================================================================================================

void setup(void)
{
    board_init();

    // sleep for 100ms
    board_delayMilliseconds(100);

    for (uint8_t i = 0; i < PITCH_LOOKUP_LENGTH; i++)
        lookupPitchRollRC[i] = (2500 + CONFIG_RC_EXPO_8 * (i * i - 25)) * i * (int32_t)CONFIG_RC_RATE_8 / 2500;

    for (uint8_t i = 0; i < THROTTLE_LOOKUP_LENGTH; i++) {
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
    for (uint8_t i = 0; i < 10; i++) {
        board_led1Toggle();
        board_led0Toggle();
        board_delayMilliseconds(50);
    }
    board_led1Off();
    board_led0Off();

    imu.init();

    mixer.init(); 

    // configure PWM/CPPM read function and max number of channels
    // these, if enabled
    for (uint8_t i = 0; i < RC_CHANS; i++)
        rcData[i] = 1502;

    previousTime = board_getMicros();

    calibratingG = CONFIG_CALIBRATING_GYRO_CYCLES;

    // trigger accelerometer calibration requirement
    haveSmallAngle = true;

} // setup

void loop(void)
{
    static uint8_t rcDelayCommand;      // this indicates the number of time (multiple of RC measurement at 50Hz) 
                                        // the sticks must be maintained to run or switch off motors

    static uint8_t rcSticks;            // this hold sticks position for command combos

    static uint32_t rcTime = 0;
    static uint32_t loopTime;
    static uint32_t calibratedAccTime;
    static int16_t  rcCommand[4];
    static bool     accCalibrated;
    static bool     armed;

    uint16_t auxState = 0;
    bool isThrottleLow = false;
    uint8_t stTmp = 0;

    if (check_and_update_timed_task(&rcTime, CONFIG_RC_LOOPTIME_USEC)) {

        computeRC();

        // ------------------ STICKS COMMAND HANDLER --------------------
        // checking sticks positions
        for (uint8_t i = 0; i < 4; i++) {
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
                if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_CE) {
                    if (armed) {
                        armed = false;
                        // Reset disarm time so that it works next time we arm the board.
                        if (disarmTime != 0)
                            disarmTime = 0;
                    }
                }
            } else {            // actions during not armed

                // GYRO calibration
                if (rcSticks == THR_LO + YAW_LO + PIT_LO + ROL_CE) {
                    calibratingG = CONFIG_CALIBRATING_GYRO_CYCLES;
                } 

                // Arm via YAW
                if ((rcSticks == THR_LO + YAW_HI + PIT_CE + ROL_CE)) {
                    if (calibratingG == 0 && accCalibrated) {
                        if (!armed) {         // arm now!
                            armed = true;
                        }
                    } else if (!armed) {
                        blinkLED(2, 255, 1);
                    }
                }

                // Calibrating Acc
                else if (rcSticks == THR_HI + YAW_LO + PIT_LO + ROL_CE)
                    calibratingA = CONFIG_CALIBRATING_ACC_CYCLES;
            }
        }

        // Check AUX switches
        for (uint8_t i = 0; i < 4; i++)
            auxState |= (rcData[AUX1 + i] < 1300) << (3 * i) | (1300 < rcData[AUX1 + i] && rcData[AUX1 + i] < 1700) 
                << (3 * i + 1) | (rcData[AUX1 + i] > 1700) << (3 * i + 2);

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

        float anglerad[3];

        imu.getEstimatedAttitude(armed, anglerad, calibratingA, calibratingG);

        angle[ROLL] = lrintf(anglerad[ROLL] * (1800.0f / M_PI));
        angle[PITCH] = lrintf(anglerad[PITCH] * (1800.0f / M_PI));

        angle[YAW] = lrintf(anglerad[YAW] * 1800.0f / M_PI + CONFIG_MAGNETIC_DECLINATION) / 10.0f;

        if (angle[YAW] < 0)
            angle[YAW] += 360;

        haveSmallAngle = abs(angle[0]) < CONFIG_SMALL_ANGLE && abs(angle[1]) < CONFIG_SMALL_ANGLE;

        // measure loop rate just afer reading the sensors
        currentTime = board_getMicros();
        previousTime = currentTime;

        // compute exponential RC commands
        computeRCExpo(rcCommand);

        // use LEDs to indicate calibration status
        if (calibratingA > 0 || calibratingG > 0) { 
            board_led0Toggle();
        } else {
            if (accCalibrated)
                board_led0Off();
            if (armed)
                board_led0On();
        }

        if (check_timed_task(calibratedAccTime)) {
            if (!haveSmallAngle) {
                accCalibrated = false; // the multi uses ACC and is not calibrated or is too much inclinated
                board_led0Toggle();
                update_timed_task(&calibratedAccTime, CONFIG_CALIBRATE_ACCTIME_USEC);
            } else {
                accCalibrated = true;
            }
        }

        // handle serial communications
        msp.com(armed, angle, mixer.motorsDisarmed, rcData);

        pidMultiWii(rcCommand);

        // prevent "yaw jump" during yaw correction
        axisPID[YAW] = constrainer(axisPID[YAW], -100 - abs(rcCommand[YAW]), +100 + abs(rcCommand[YAW]));

        mixer.writeMotors(armed, axisPID, rcCommand, rcData);

    } // if (check_and_update_timed_task(&loopTime, CONFIG_IMU_LOOPTIME_USEC))

} // loop()

} // extern "C"
