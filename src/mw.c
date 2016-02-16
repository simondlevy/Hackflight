/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#define I2C_DEVICE (I2CDEV_2)

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "stm32f10x_conf.h"

#include "board/drv_adc.h"
#include "board/drv_serial.h"
#include "board/drv_gpio.h"
#include "board/drv_system.h"
#include "board/drv_pwm.h"
#include "board/drv_spi.h"
#include "board/drv_i2c.h"
#include "board/revision.h"

#include "mixer.h"
#include "sensors.h"
#include "axes.h"
#include "mw.h"
#include "config.h"
#include "utils.h"

#define PITCH_LOOKUP_LENGTH     7
#define THROTTLE_LOOKUP_LENGTH 12

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

// Globals
bool     useSmallAngle;
bool     armed;
uint32_t currentTime = 0;
int16_t  rcData[RC_CHANS];       // interval [1000;2000]
int16_t  axisPID[3];

static int16_t motor[4];
static int16_t motor_disarmed[4];


static int16_t  rcCommand[4];   // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW

static uint8_t accCalibrated;

static uint8_t dynP8[3], dynI8[3], dynD8[3];

static int16_t lookupPitchRollRC[PITCH_LOOKUP_LENGTH];   // lookup table for expo & RC rate PITCH+ROLL
static int16_t lookupThrottleRC[THROTTLE_LOOKUP_LENGTH];   // lookup table for expo & mid THROTTLE

static bool check_timed_task(uint32_t usec) {

    return (int32_t)(currentTime - usec) >= 0;
}

static void update_timed_task(uint32_t * usec, uint32_t period) 
{
    *usec = currentTime + period;
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

    tmp = constrain(rcData[THROTTLE], CONFIG_MINCHECK, 2000);
    tmp = (uint32_t)(tmp - CONFIG_MINCHECK) * 1000 / (2000 - CONFIG_MINCHECK);       // [MINCHECK;2000] -> [0;1000]
    tmp2 = tmp / 100;
    rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp - tmp2 * 100) * (lookupThrottleRC[tmp2 + 1] - 
            lookupThrottleRC[tmp2]) / 100;    // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]

    if (calibratingA > 0 || calibratingG > 0) {      // Calibration phasis
        LED0_TOGGLE;
    } else {
        if (accCalibrated)
            LED0_OFF;
        if (armed)
            LED0_ON;
    }

    if (check_timed_task(calibratedAccTime)) {
        if (!useSmallAngle) {
            accCalibrated = 0; // the multi uses ACC and is not calibrated or is too much inclinated
            LED0_TOGGLE;
            update_timed_task(&calibratedAccTime, CONFIG_CALIBRATE_ACCTIME_USEC);
            //calibratedAccTime = currentTime + CONFIG_CALIBRATE_ACCTIME_USEC;
        } else {
            accCalibrated = 1;
        }
    }

    extern void mspCom(int16_t motor[4], int16_t motor_disarmed[4]);
    mspCom(motor, motor_disarmed);
}

static void computeRC(void)
{
    uint16_t capture;
    int i, chan;

    static int16_t rcDataAverage[8][4];
    static int rcAverageIndex = 0;

    for (chan = 0; chan < 8; chan++) {

        capture = pwmRead(CONFIG_RCMAP[chan]);

        // validate input
        if (capture < PULSE_MIN || capture > PULSE_MAX)
            capture = CONFIG_MIDRC;
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
    // Time of automatic disarm when "Don't spin the motors when armed" is enabled.
    static uint32_t disarmTime;

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
            errorAngle = constrain(2 * rcCommand[axis], -((int)CONFIG_MAX_ANGLE_INCLINATION), 
                    + CONFIG_MAX_ANGLE_INCLINATION) - angle[axis] + CONFIG_ANGLE_TRIM[axis];
            PTermACC = errorAngle * CONFIG_LEVEL_P / 100; 
            // 32 bits is needed for calculation: errorAngle*CONFIG_LEVEL_P could exceed 32768   
            // 16 bits is ok for result
            PTermACC = constrain(PTermACC, -CONFIG_LEVEL_D * 5, + CONFIG_LEVEL_D * 5);

            errorAngleI[axis] = constrain(errorAngleI[axis] + errorAngle, -10000, +10000); // WindUp
            ITermACC = (errorAngleI[axis] * CONFIG_LEVEL_I) >> 12;
        }
        if (CONFIG_HORIZON_MODE || axis == 2) { // MODE relying on GYRO or YAW axis
            error = (int32_t)rcCommand[axis] * 10 * 8 / CONFIG_AXIS_P[axis];
            error -= gyroData[axis];

            PTermGYRO = rcCommand[axis];

            errorGyroI[axis] = constrain(errorGyroI[axis] + error, -16000, +16000); // WindUp
            if ((abs(gyroData[axis]) > 640) || ((axis == YAW) && (abs(rcCommand[axis]) > 100)))
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

        PTerm -= (int32_t)gyroData[axis] * dynP8[axis] / 10 / 8; // 32 bits is needed for calculation
        delta = gyroData[axis] - lastGyro[axis];
        lastGyro[axis] = gyroData[axis];
        deltaSum = delta1[axis] + delta2[axis] + delta;
        delta2[axis] = delta1[axis];
        delta1[axis] = delta;
        DTerm = (deltaSum * dynD8[axis]) / 32;
        axisPID[axis] = PTerm + ITerm - DTerm;
    }
}

// ===============================================================================================

bool check_and_update_timed_task(uint32_t * usec, uint32_t period) 
{
    bool result = (int32_t)(currentTime - *usec) >= 0;

    if (result)
        update_timed_task(usec, period);

    return result;
}

void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat)
{
    uint8_t i, r;

    for (r = 0; r < repeat; r++) {
        for (i = 0; i < num; i++) {
            LED0_TOGGLE;            // switch LEDPIN state
            delay(wait);
        }
        delay(60);
    }
}

void setup(void)
{
    int i;

    // determine hardware revision based on clock frequency
    int hw_revision = 0;
    if (hse_value == 8000000)
        hw_revision = NAZE32;
    else if (hse_value == 12000000)
        hw_revision = NAZE32_REV5;

    systemInit(hw_revision);

    // sleep for 100ms
    delay(100);

    if (spiInit() == SPI_DEVICE_MPU && hw_revision == NAZE32_REV5)
        hw_revision = NAZE32_SP;

    if (hw_revision != NAZE32_SP)
        i2cInit(I2C_DEVICE);

    adcInit(hw_revision);

    initSensors(hw_revision);


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


    LED1_ON;
    LED0_OFF;
    for (i = 0; i < 10; i++) {
        LED1_TOGGLE;
        LED0_TOGGLE;
        delay(50);
    }
    LED0_OFF;
    LED1_OFF;

    imuInit(); 
    mixerInit(motor_disarmed); 

    pwmInit(CONFIG_FAILSAFE_DETECT_THRESHOLD, CONFIG_PWM_FILTER, CONFIG_USE_CPPM, CONFIG_MOTOR_PWM_RATE,
            CONFIG_FAST_PWM, CONFIG_PWM_IDLE_PULSE);

    // configure PWM/CPPM read function and max number of channels
    // these, if enabled
    for (i = 0; i < RC_CHANS; i++)
        rcData[i] = 1502;

    calibratingG = CONFIG_CALIBRATING_GYRO_CYCLES;

    // trigger accelerometer calibration requirement
    useSmallAngle = true;

    armed = false;
}

#define GYRO_I_MAX 256

void loop(void)
{
    static uint8_t rcDelayCommand;      // this indicates the number of time (multiple of RC measurement at 50Hz) 
    // the sticks must be maintained to run or switch off motors
    static uint8_t rcSticks;            // this hold sticks position for command combos
    uint8_t stTmp = 0;
    int i;
    static uint32_t rcTime = 0;
    static int16_t initialThrottleHold;
    static uint32_t loopTime;
    uint16_t auxState = 0;
    bool isThrottleLow = false;

    static uint8_t alt_hold_mode;

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
        if (rcData[AUX1] > 1300) {
            auxState = 1;
            if (rcData[AUX1] > 1700)
                auxState = 2;
        }

        // Switch to alt-hold when switch moves to position 1 or 2
        if (auxState > 0) {
            if (!alt_hold_mode) {
                alt_hold_mode = 1;
                AltHold = EstAlt;
                initialThrottleHold = rcCommand[THROTTLE];
                errorVelocityI = 0;
                AltPID = 0;
            }
        }
        else {
            alt_hold_mode = 0;
        }

    } else {                        // not in rc loop
        static int taskOrder = 0;   // never call all function in the same loop, to avoid high delay spikes
        switch (taskOrder) {
            case 0:
                taskOrder++;
                if (sonar_available) {
                    Sonar_update();
                    break;
                }
            case 1:
                taskOrder++;
                if (baro_available) {
                    Baro_update();
                    break;
                }
            case 2:
                taskOrder++;
                if (baro_available && sonar_available) {
                    getEstimatedAltitude();
                    break;
                }
            case 3:
                // if GPS feature is enabled, gpsThread() will be called at some intervals to check for stuck
                // hardware, wrong baud rates, init GPS if needed, etc. Don't use SENSOR_GPS here as gpsThread() 
                // can and will change this based on available hardware
                taskOrder++;
            case 4:
                taskOrder = 0;
                break;
        }
    }
    
    currentTime = micros();

    if (check_and_update_timed_task(&loopTime, CONFIG_IMU_LOOPTIME_USEC)) {

        computeIMU();

        // Measure loop rate just afer reading the sensors
        currentTime = micros();

        // non IMU critical, temeperatur, serialcom
        annexCode();

        if (alt_hold_mode) {
            static uint8_t isAltHoldChanged = 0;
            if (CONFIG_ALT_HOLD_FAST_CHANGE) {
                // rapid alt changes
                if (abs(rcCommand[THROTTLE] - initialThrottleHold) > CONFIG_ALT_HOLD_THROTTLE_NEUTRAL) {
                    errorVelocityI = 0;
                    isAltHoldChanged = 1;
                    rcCommand[THROTTLE] += (rcCommand[THROTTLE] > initialThrottleHold) 
                        ? -CONFIG_ALT_HOLD_THROTTLE_NEUTRAL : CONFIG_ALT_HOLD_THROTTLE_NEUTRAL;
                } else {
                    if (isAltHoldChanged) {
                        AltHold = EstAlt;
                        isAltHoldChanged = 0;
                    }
                    rcCommand[THROTTLE] = constrain(initialThrottleHold + AltPID, 
                            CONFIG_MINTHROTTLE, CONFIG_MAXTHROTTLE);
                }
            } else {
                // slow alt changes for apfags
                if (abs(rcCommand[THROTTLE] - initialThrottleHold) > CONFIG_ALT_HOLD_THROTTLE_NEUTRAL) {
                    // set velocity proportional to stick movement +100 throttle gives ~ +50 cm/s
                    setVelocity = (rcCommand[THROTTLE] - initialThrottleHold) / 2;
                    velocityControl = true;
                    isAltHoldChanged = 1;
                } else if (isAltHoldChanged) {
                    AltHold = EstAlt;
                    velocityControl = false;
                    isAltHoldChanged = 0;
                }
                rcCommand[THROTTLE] = constrain(initialThrottleHold + AltPID, CONFIG_MINTHROTTLE, CONFIG_MAXTHROTTLE);
            }
        }


        if (CONFIG_THROTTLE_CORRECTION_VALUE && CONFIG_HORIZON_MODE) {
            rcCommand[THROTTLE] += throttleAngleCorrection;
        }

        pidMultiWii();
        mixTable(rcCommand, motor, motor_disarmed);
        writeMotors(motor);
    }
}
