extern "C" {

#include <breezystm32.h>
#include <math.h>

#include "../board.hpp"

#define USE_CPPM       1
#define PWM_FILTER     0     // 0 or 1
#define FAST_PWM       0     // 0 or 1
#define MOTOR_PWM_RATE 400
#define PWM_IDLE_PULSE 1000  // standard PWM in usec for brushless ESC

extern serialPort_t * Serial1;

void Board::imuInit(uint16_t *acc1G, float * gyroScale)
{
    mpu6050_init(false, acc1G, gyroScale);
}

void Board::imuRead(int16_t accADC[3], int16_t gyroADC[3])
{
    mpu6050_read_accel(accADC);
    mpu6050_read_gyro(gyroADC);
}

void Board::init(void)
{
    i2cInit(I2CDEV_2);
    pwmInit(USE_CPPM, PWM_FILTER, FAST_PWM, MOTOR_PWM_RATE, PWM_IDLE_PULSE);
}

void Board::checkReboot(bool pendReboot)
{
    if (pendReboot)
        systemReset(false); // noreturn
}

void Board::delayMilliseconds(uint32_t msec)
{
    delay(msec);
}

uint32_t Board::getMicros()
{
    return micros();
}

void Board::ledGreenOff(void)
{
    digitalHi(LED0_GPIO, LED0_PIN);
}

void Board::ledGreenOn(void)
{
    digitalLo(LED0_GPIO, LED0_PIN);
}

void Board::ledGreenToggle(void)
{
    digitalToggle(LED0_GPIO, LED0_PIN);
}

void Board::ledRedOff(void)
{
    digitalHi(LED1_GPIO, LED1_PIN);
}

void Board::ledRedOn(void)
{
    digitalLo(LED1_GPIO, LED1_PIN);
}

void Board::ledRedToggle(void)
{
    digitalToggle(LED1_GPIO, LED1_PIN);
}

uint16_t Board::readPWM(uint8_t chan)
{
    return pwmRead(chan);
}

void Board::reboot(void)
{
    systemReset(true);      // reboot to bootloader
}

uint8_t Board::serialAvailableBytes(void)
{
    return serialTotalBytesWaiting(Serial1);
}

uint8_t Board::serialReadByte(void)
{
    return serialRead(Serial1);
}

void Board::serialWriteByte(uint8_t c)
{
    serialWrite(Serial1, c);
}

void Board::writeMotor(uint8_t index, uint16_t value)
{
    pwmWriteMotor(index, value);
}

} // extern "C"
