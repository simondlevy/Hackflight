#include <Arduino.h>
#include <Servo.h>
#include <PulsePosition.h>
#include <MPU6050.h>
#include <i2c_t3.h>

#include <math.h>

#include "mw.h"

static const int PPM_PIN = 6;

static const int ESC_PINS[4] = {9, 10, 11, 12};
static const int ESC_USEC_MIN = 800;

// unused =========================================================

void board_checkReboot(bool pendReboot)
{
}

uint16_t board_getI2cErrorCounter(void)
{
    return 0;
}

void board_reboot(void)
{
}

// everything but IMU ==============================================

Servo escs[4];

PulsePositionInput ppm;

void board_delayMilliseconds(uint32_t msec)
{
    delay(msec);
}

uint32_t board_getMicros()
{
    return micros();
}

void board_init(void)
{
    Serial.begin(115200);

    pinMode(13, OUTPUT);  // LED

    for (int k=0; k<4; ++k) {
        escs[k].attach(ESC_PINS[k]);
        escs[k].writeMicroseconds(ESC_USEC_MIN);
    }

    ppm.begin(PPM_PIN);
}

static bool ledState;

void board_led0Off(void)
{
    ledState = false;
    digitalWriteFast(13, LOW);
}

void board_led0On(void)
{
    ledState = true;
    digitalWriteFast(13, HIGH);
}

void board_led0Toggle(void)
{
    ledState = !ledState;
    digitalWriteFast(13, ledState ? HIGH : LOW);
}


void board_led1Off(void)
{
}

void board_led1On(void)
{
}

void board_led1Toggle(void)
{
}

uint16_t board_pwmRead(uint8_t chan)
{
    return ppm.read(chan+1);
}

uint8_t board_serialAvailable(void)
{
    return Serial.available();
}

uint8_t board_serialRead(void)
{
    return Serial.read();
}

void board_serialWrite(uint8_t c)
{
    Serial.write(c);
}

void board_writeMotor(uint8_t index, uint16_t value)
{
    escs[index].writeMicroseconds(value);
}

// IMU ===============================================================


MPU6050 accelgyro;


void board_imuInit(uint16_t *acc1G, float * gyroScale)
{
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_INT, I2C_RATE_400);

    accelgyro.initialize();

    accelgyro.setRate(0x00);
    accelgyro.setClockSource(3);
    accelgyro.setDLPFMode(3);
    accelgyro.setFullScaleGyroRange(3);
    accelgyro.setFullScaleAccelRange(2);
    accelgyro.setInterruptMode(0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0);
    accelgyro.setIntEnabled(0x01);

    // Set acc1G. Modified once by mpu6050CheckRevision for old (hopefully nonexistent outside of clones) parts
    *acc1G = 512 * 8;

    // 16.4 dps/lsb scalefactor for all Invensense devices
    *gyroScale = (4.0f / 16.4f) * (M_PI / 180.0f) * 0.000001f;
}

void board_imuRead(int16_t accADC[3], int16_t gyroADC[3])
{  
    accelgyro.getMotion6(&accADC[0], &accADC[1], &accADC[2], &gyroADC[0], &gyroADC[1], &gyroADC[2]);

    gyroADC[0] /= 4;
    gyroADC[1] /= 4;
    gyroADC[2] /= 4;
}

