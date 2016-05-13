extern "C" {

#include <breezystm32.h>
#include <math.h>

#include "board.hpp"

#define USE_CPPM       1
#define PWM_FILTER     0     // 0 or 1
#define FAST_PWM       0     // 0 or 1
#define MOTOR_PWM_RATE 400
#define PWM_IDLE_PULSE 1000  // standard PWM in usec for brushless ESC

extern serialPort_t * Serial1;

void board_imuInit(uint16_t *acc1G, float * gyroScale)
{
    mpu6050_init(false, acc1G, gyroScale);
}

void board_imuRead(int16_t accADC[3], int16_t gyroADC[3])
{
    mpu6050_read_accel(accADC);
    mpu6050_read_gyro(gyroADC);
}

void Board::init(void)
{
    i2cInit(I2CDEV_2);
    pwmInit(USE_CPPM, PWM_FILTER, FAST_PWM, MOTOR_PWM_RATE, PWM_IDLE_PULSE);
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

void board_led0Off(void)
{
    LED0_OFF;
}

void board_led0On(void)
{
    LED0_ON;
}

void board_led0Toggle(void)
{
    LED0_TOGGLE;
}

void board_led1Off(void)
{
    LED1_OFF;
}

void board_led1On(void)
{
    LED1_ON;
}

void board_led1Toggle(void)
{
    LED1_TOGGLE;
}

uint16_t board_pwmRead(uint8_t chan)
{
    return pwmRead(chan);
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

} // extern "C"
