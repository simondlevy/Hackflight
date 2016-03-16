#include <breezystm32.h>

#include "board.h"

#define USE_CPPM                             1
#define PWM_FILTER                           0     /* 0 or 1 */
#define FAST_PWM                             0     /* 0 or 1 */
#define MOTOR_PWM_RATE                       400
#define PWM_IDLE_PULSE                       1000  /* standard PWM in usec for brushless ESC */

extern serialPort_t * Serial1;

void board_imuInit(uint8_t lpf, uint16_t * acc1G, float * gyroScale)
{
    mpu6050_init(false, lpf, acc1G, gyroScale);
}

void board_imuReadAccel(int16_t * data)
{
    mpu6050_read_accel(data);
}

void board_imuReadGyro(int16_t * data)
{
    mpu6050_read_gyro(data);
}

void board_checkReboot(bool pendReboot)
{
    if (pendReboot)
        systemReset(false); // noreturn
}

void board_pwmInit(void)
{
    pwmInit(USE_CPPM, PWM_FILTER, FAST_PWM, MOTOR_PWM_RATE, PWM_IDLE_PULSE);
}

uint16_t board_pwmRead(uint8_t chan)
{
    return pwmRead(chan);
}

void board_reboot(void)
{
    systemReset(true);      // reboot to bootloader
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

bool board_serialAvailable(void)
{
    return serialTotalBytesWaiting(Serial1) > 0;
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
