/*
   Streaming support for MPU9250 IMU

   Adapted from http://arduinolearning.com/code/arduino-mpu-9250-example.php

   Copyright (C) 2021 Simon D. Levy

   MIT License
 */


#include <Wire.h>

static const uint8_t MPU9250_ADDRESS = 0x68;

static const uint8_t MPU9250_INTERRUPT_STATUS = 0x3A;

static const uint8_t GYRO_FULL_SCALE_250_DPS = 0x00;
static const uint8_t GYRO_FULL_SCALE_500_DPS = 0x08;
static const uint8_t GYRO_FULL_SCALE_1000_DPS = 0x10;
static const uint8_t GYRO_FULL_SCALE_2000_DPS = 0x18;

static const uint8_t ACC_FULL_SCALE_2_G = 0x00;
static const uint8_t ACC_FULL_SCALE_4_G = 0x08;
static const uint8_t ACC_FULL_SCALE_8_G = 0x10;
static const uint8_t ACC_FULL_SCALE_16_G = 0x18;

// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
static void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
    // Set register address
    Wire.beginTransmission(Address);
    Wire.write(Register);
    Wire.endTransmission();

    // Read Nbytes
    Wire.requestFrom(Address, Nbytes); 
    uint8_t index=0;
    while (Wire.available())
        Data[index++]=Wire.read();
}


// Write a byte (Data) in device (Address) at register (Register)
static void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
    // Set register address
    Wire.beginTransmission(Address);
    Wire.write(Register);
    Wire.write(Data);
    Wire.endTransmission();
}

void stream_startMpu9250(void)
{
    // Set accelerometers low pass filter at 5Hz
    I2CwriteByte(MPU9250_ADDRESS, 29, 0x06);

    // Set gyroscope low pass filter at 5Hz
    I2CwriteByte(MPU9250_ADDRESS, 26, 0x06);

    // Configure gyroscope range
    I2CwriteByte(MPU9250_ADDRESS, 27, GYRO_FULL_SCALE_1000_DPS);

    // Configure accelerometers range
    I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_4_G);
}


void stream_updateMpu9250(void)
{
    // Read accelerometer and gyroscoe
    uint8_t Buf[14] = {};
    I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);

    // Create 16 bits values from 8 bits data

    // Accelerometer
    int16_t ax=-(Buf[0]<<8 | Buf[1]);
    int16_t ay=-(Buf[2]<<8 | Buf[3]);
    int16_t az=Buf[4]<<8 | Buf[5];

    // Gyroscope
    int16_t gx=-(Buf[8]<<8 | Buf[9]);
    int16_t gy=-(Buf[10]<<8 | Buf[11]);
    int16_t gz=Buf[12]<<8 | Buf[13];

    uint8_t ready = 0;
    I2Cread(MPU9250_ADDRESS, 0x3A, 1, &ready);

    printf("%d\n", ready);
    //printf("%d %d %d\n", ax, ay, az);
}
