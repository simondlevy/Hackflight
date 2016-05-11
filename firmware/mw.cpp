extern "C" {

#include "mw.hpp"
#include "imu.hpp"
#include "mixer.hpp"
#include "msp.hpp"
#include "pid.hpp"
#include "rc.hpp"

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

static bool check_timed_task(uint32_t usec, uint32_t currentTime) 
{

    return (int32_t)(currentTime - usec) >= 0;
}

static void update_timed_task(uint32_t * usec, uint32_t period, uint32_t currentTime) 
{
    *usec = currentTime + period;
}

static bool check_and_update_timed_task(uint32_t * usec, uint32_t period, uint32_t currentTime) 
{
    bool result = (int32_t)(currentTime - *usec) >= 0;

    if (result)
        update_timed_task(usec, period, currentTime);

    return result;
}


// Objects we use

IMU   imu;
RC    rc;
Mixer mixer;
PID   pid;
MSP   msp;

// values initialized in setup()

static uint16_t calibratingG;
static bool     haveSmallAngle;
static uint32_t previousTime;

void setup(void)
{
    board_init();

    // sleep for 100ms
    board_delayMilliseconds(100);

    // flash the LEDs to indicate startup
    board_led1On();
    board_led0Off();
    for (uint8_t i = 0; i < 10; i++) {
        board_led1Toggle();
        board_led0Toggle();
        board_delayMilliseconds(50);
    }
    board_led1Off();
    board_led0Off();

    // initialize our RC, IMU, mixer, and PID controller
    rc.init();
    imu.init();
    mixer.init(); 
    pid.init();

    // set initial time
    previousTime = board_getMicros();

    // always do gyro calibration at startup
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
    static bool     accCalibrated;
    static bool     armed;
    static int16_t  angle[3];
    static int16_t  axisPID[3];
    static int32_t  errorGyroI[3];
    static int32_t  errorAngleI[2];
    static uint16_t calibratingA;
    static uint32_t currentTime;
    static uint32_t disarmTime = 0;

    uint16_t auxState = 0;
    uint8_t stTmp = 0;

    if (check_and_update_timed_task(&rcTime, CONFIG_RC_LOOPTIME_USEC, currentTime)) {

        rc.compute();

        // checking sticks positions
        for (uint8_t i = 0; i < 4; i++) {
            stTmp >>= 2;
            if (rc.data[i] > CONFIG_MINCHECK)
                stTmp |= 0x80;  // check for MIN
            if (rc.data[i] < CONFIG_MAXCHECK)
                stTmp |= 0x40;  // check for MAX
        }
        if (stTmp == rcSticks) {
            if (rcDelayCommand < 250)
                rcDelayCommand++;
        } else
            rcDelayCommand = 0;
        rcSticks = stTmp;

        // when landed, reset integral component of PID
        if ((rc.data[THROTTLE] < CONFIG_MINCHECK)) {
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
            auxState |= (rc.data[AUX1 + i] < 1300) << (3 * i) | (1300 < rc.data[AUX1 + i] && rc.data[AUX1 + i] < 1700) 
                << (3 * i + 1) | (rc.data[AUX1 + i] > 1700) << (3 * i + 2);

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

    if (check_and_update_timed_task(&loopTime, CONFIG_IMU_LOOPTIME_USEC, currentTime)) {

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
        rc.computeExpo();

        // use LEDs to indicate calibration status
        if (calibratingA > 0 || calibratingG > 0) { 
            board_led0Toggle();
        } else {
            if (accCalibrated)
                board_led0Off();
            if (armed)
                board_led0On();
        }

        if (check_timed_task(calibratedAccTime, currentTime)) {
            if (!haveSmallAngle) {
                accCalibrated = false; // the multi uses ACC and is not calibrated or is too much inclinated
                board_led0Toggle();
                update_timed_task(&calibratedAccTime, CONFIG_CALIBRATE_ACCTIME_USEC, currentTime);
            } else {
                accCalibrated = true;
            }
        }

        // handle serial communications
        msp.com(armed, angle, mixer.motorsDisarmed, rc.data);

        // run PID controller 
        pid.compute(rc.command, angle, imu.gyroADC, axisPID, errorGyroI, errorAngleI);

        // prevent "yaw jump" during yaw correction
        axisPID[YAW] = constrain(axisPID[YAW], -100 - abs(rc.command[YAW]), +100 + abs(rc.command[YAW]));

        mixer.writeMotors(armed, axisPID, rc.command, rc.data);

    } // IMU update

} // loop()

} // extern "C"
