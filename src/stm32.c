#include <breezystm32.h>

#include "board.h"

#define USE_CPPM                             1
#define PWM_FILTER                           0     /* 0 or 1 */
#define FAST_PWM                             0     /* 0 or 1 */
#define MOTOR_PWM_RATE                       400
#define PWM_IDLE_PULSE                       1000  /* standard PWM in usec for brushless ESC */

extern serialPort_t * Serial1;

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

void board_i2cInit(void)
{
    i2cInit(I2CDEV_2);
}

void board_imuInit(uint16_t * acc1G, float * gyroScale)
{
    mpu6050_init(false, acc1G, gyroScale);
}

void board_imuReadAccel(int16_t * data)
{
    mpu6050_read_accel(data);
}

void board_imuReadGyro(int16_t * data)
{
    mpu6050_read_gyro(data);
}

void board_ledOff(void)
{
    LED0_OFF;
}

void board_ledOn(void)
{
    LED0_ON;
}

void board_ledToggle(void)
{
    LED0_TOGGLE;
}

void board_pwmInit(void)
{
    pwmInit(USE_CPPM, PWM_FILTER, FAST_PWM, MOTOR_PWM_RATE, PWM_IDLE_PULSE);
}

uint16_t board_pwmRead(uint8_t chan, uint16_t oob_default)
{
    return pwmRead(chan, oob_default);
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
