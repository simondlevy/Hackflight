extern "C" {

#include "mw.hpp"

#ifndef PRINTF
#define PRINTF printf
#endif

#include <math.h>

// Objects we use

Board board;
IMU   imu;
RC    rc;
Mixer mixer;
PID   pid;
MSP   msp;

// utilities 

static void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat)
{
    for (uint8_t r = 0; r < repeat; r++) {
        for (uint8_t i = 0; i < num; i++) {
            board.led0Toggle();            // switch LEDPIN state
            board.delayMilliseconds(wait);
        }
        board.delayMilliseconds(60);
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


// values initialized in setup()

static uint16_t calibratingG;
static bool     haveSmallAngle;
static uint32_t previousTime;

void setup(void)
{
    board.init();

    // sleep for 100ms
    board.delayMilliseconds(100);

    // flash the LEDs to indicate startup
    board.led1On();
    board.led0Off();
    for (uint8_t i = 0; i < 10; i++) {
        board.led1Toggle();
        board.led0Toggle();
        board.delayMilliseconds(50);
    }
    board.led1Off();
    board.led0Off();

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
    static uint32_t rcTime = 0;
    static uint32_t loopTime;
    static uint32_t calibratedAccTime;
    static bool     accCalibrated;
    static bool     armed;
    static uint16_t calibratingA;
    static uint32_t currentTime;
    static uint32_t disarmTime = 0;

    if (check_and_update_timed_task(&rcTime, CONFIG_RC_LOOPTIME_USEC, currentTime)) {

        // update RC channels
        rc.update();

        // when landed, reset integral component of PID
        if (rc.throttleIsDown()) 
            pid.resetIntegral();

        if (rc.changed()) {

            if (armed) {      // actions during armed

                // Disarm on throttle down + yaw
                if (rc.sticks == THR_LO + YAW_LO + PIT_CE + ROL_CE) {
                    if (armed) {
                        armed = false;
                        // Reset disarm time so that it works next time we arm the board.
                        if (disarmTime != 0)
                            disarmTime = 0;
                    }
                }
            } else {            // actions during not armed

                // GYRO calibration
                if (rc.sticks == THR_LO + YAW_LO + PIT_LO + ROL_CE) {
                    calibratingG = CONFIG_CALIBRATING_GYRO_CYCLES;
                } 

                // Arm via YAW
                if ((rc.sticks == THR_LO + YAW_HI + PIT_CE + ROL_CE)) {
                    if (calibratingG == 0 && accCalibrated) {
                        if (!armed) {         // arm now!
                            armed = true;
                        }
                    } else if (!armed) {
                        blinkLED(2, 255, 1);
                    }
                }

                // Calibrating Acc
                else if (rc.sticks == THR_HI + YAW_LO + PIT_LO + ROL_CE)
                    calibratingA = CONFIG_CALIBRATING_ACC_CYCLES;
            }

        } // if rc.changed()

    } else {                    // not in rc loop

        static int taskOrder;   // never call all functions in the same loop, to avoid high delay spikes

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

        imu.update(armed, calibratingA, calibratingG);

        haveSmallAngle = abs(imu.angle[0]) < CONFIG_SMALL_ANGLE && abs(imu.angle[1]) < CONFIG_SMALL_ANGLE;

        // measure loop rate just afer reading the sensors
        currentTime = board_getMicros();
        previousTime = currentTime;

        // compute exponential RC commands
        rc.computeExpo();

        // use LEDs to indicate calibration status
        if (calibratingA > 0 || calibratingG > 0) { 
            board.led0Toggle();
        } else {
            if (accCalibrated)
                board.led0Off();
            if (armed)
                board.led0On();
        }

        if (check_timed_task(calibratedAccTime, currentTime)) {
            if (!haveSmallAngle) {
                accCalibrated = false; // the multi uses ACC and is not calibrated or is too much inclinated
                board.led0Toggle();
                update_timed_task(&calibratedAccTime, CONFIG_CALIBRATE_ACCTIME_USEC, currentTime);
            } else {
                accCalibrated = true;
            }
        }

        // handle serial communications
        msp.update(armed, &imu, &mixer, rc.data);

        // update PID controller 
        pid.update(&rc, &imu);

        // update mixer
        mixer.update(armed, &pid, &rc);

    } // IMU update

} // loop()

} // extern "C"
