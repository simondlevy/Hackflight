/*
   board.cpp : implementation of board-specific routines

   This implemenation is for Teensy 3.1 / 3.2 with MPU9250 IMU. 

   MPU9250 code adapted from https://github.com/kriswiner/MPU-9250

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <math.h>
#include <stdint.h>
#include <stdarg.h>

#include <Arduino.h>
#include <PulsePosition.h>
#include <I2Cdev.h>

#include "board.hpp"
#include "rc.hpp"

#define IMU_LOOPTIME_USEC       3500
#define CALIBRATING_GYRO_MSEC   3500

#define PPM_INPUT_PIN           5

#define MPU9250_ADDRESS  0x68  

#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define ACCEL_XOUT_H     0x3B
#define GYRO_XOUT_H      0x43
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71

// Set initial input parameters
enum Ascale {
    AFS_2G = 0,
    AFS_4G,
    AFS_8G,
    AFS_16G
};

enum Gscale {
    GFS_250DPS = 0,
    GFS_500DPS,
    GFS_1000DPS,
    GFS_2000DPS
};

// Specify sensor full scale
static uint8_t Gscale = GFS_2000DPS;
static uint8_t Ascale = AFS_8G;

// https://www.tindie.com/products/onehorse/dc-motor-controller-board-for-teensy-31-/
static const uint8_t MOTOR_PINS[4] = {23, 4, 3, 22};

static PulsePositionInput ppmIn;


// I^2C utility functions --------------------------------------------------------------

static void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
    Wire.beginTransmission(address);  // Initialize the Tx buffer
    Wire.write(subAddress);           // Put slave register address in Tx buffer
    Wire.write(data);                 // Put data in Tx buffer
    Wire.endTransmission();           // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
    uint8_t data; // `data` will store the register data   
    Wire.beginTransmission(address);         // Initialize the Tx buffer
    Wire.write(subAddress);                  // Put slave register address in Tx buffer
    Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
    //  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
    //  Wire.requestFrom(address, 1);  // Read one byte from slave register address 
    Wire.requestFrom(address, (size_t) 1);  // Read one byte from slave register address 
    data = Wire.read();                      // Fill Rx buffer with result
    return data;                             // Return data read from slave register
}

static void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
    Wire.beginTransmission(address);   // Initialize the Tx buffer
    Wire.write(subAddress);            // Put slave register address in Tx buffer
    Wire.endTransmission(I2C_NOSTOP);  // Send the Tx buffer, but send a restart to keep connection alive
    //  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
    //        Wire.requestFrom(address, count);  // Read bytes from slave register address 
    Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address 
    while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}

// IMU utility functions -----------------------------------------------------------------------

static void readAccelData(int16_t * destination)
{
    uint8_t rawData[6];  // x/y/z accel register data stored here
    readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
    destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
    destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}


static void readGyroData(int16_t * destination)
{
    uint8_t rawData[6];  // x/y/z gyro register data stored here
    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
    destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}

// Board method implementations -------------------------------------------------------------------

void Board::debug(char c)
{
    Serial1.write(c);
}

void Board::imuInit(uint16_t & acc1G, float & gyroScale)
{
    // wake up device
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
    delay(100); // Wait for all registers to reset 

    // get stable time source
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
    delay(200); 

    // Configure Gyro and Thermometer
    // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
    // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
    // be higher than 1 / 0.0059 = 170 Hz
    // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
    // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
    writeByte(MPU9250_ADDRESS, CONFIG, 0x03);  

    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate 
    // determined inset in CONFIG above

    // Set gyroscope full scale range
    // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
    uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
    // c = c & ~0xE0; // Clear self-test bits [7:5] 
    c = c & ~0x02; // Clear Fchoice bits [1:0] 
    c = c & ~0x18; // Clear AFS bits [4:3]
    c = c | Gscale << 3; // Set full scale range for the gyro
    // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register

    // Set accelerometer full-scale range configuration
    c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
    // c = c & ~0xE0; // Clear self-test bits [7:5] 
    c = c & ~0x18;  // Clear AFS bits [4:3]
    c = c | Ascale << 3; // Set full scale range for the accelerometer 
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
    // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
    c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
    c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
    c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
    // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
    // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
    // can join the I2C bus and all can be controlled by the Arduino as master
    writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);    
    writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
 
    // Accel scale 8g (4096 LSB/g)
    acc1G = 4096;

    // 16.4 dps/lsb scalefactor for all Invensense devices
    gyroScale = (1.0f / 16.4f) * (M_PI / 180.0f);
}

void Board::imuRead(int16_t accADC[3], int16_t gyroADC[3])
{
    readAccelData(accADC);  
    readGyroData(gyroADC);
}

void Board::init(uint32_t & looptimeMicroseconds, uint32_t & calibratingGyroMsec)
{
    // Set up LED
    pinMode(13, OUTPUT);
    
    // Set up I^2C
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_INT, I2C_RATE_400);
    
    // Set up PPM receiver
    ppmIn.begin(PPM_INPUT_PIN);

    // Set up serial communication over USB
    Serial.begin(115200);

    // XXX these values should probably be #define'd generally for all physical (non-simulated) boards
    looptimeMicroseconds = IMU_LOOPTIME_USEC;
    calibratingGyroMsec  = CALIBRATING_GYRO_MSEC;
}

void Board::delayMilliseconds(uint32_t msec)
{
    delay(msec);
}

uint32_t Board::getMicros()
{
    return micros();
}

void Board::ledSetState(uint8_t id, bool state)
{
    digitalWrite(13, state); // we only have one LED
}

uint16_t Board::readPWM(uint8_t chan)
{
    return (uint16_t)ppmIn.read(chan+1);
}

uint8_t Board::serialAvailableBytes(void)
{
    return Serial.available();
}

uint8_t Board::serialReadByte(void)
{
    return Serial.read();
}

void Board::serialWriteByte(uint8_t c)
{
    Serial.write(c);
}

void Board::writeMotor(uint8_t index, uint16_t value)
{  
  analogWrite(MOTOR_PINS[index], map(value, CONFIG_PWM_MIN, CONFIG_PWM_MAX, 0, 255));
}

// Non-essentials ----------------------------------------------------------------

void Board::reboot(void)
{
}

bool Board::sonarInit(uint8_t index) 
{
    index = index; // avoid compiler warning about unused variable
    return false;
}

void Board::sonarUpdate(uint8_t index)
{
    index = index; // avoid compiler warning about unused variable
}

uint16_t Board::sonarGetDistance(uint8_t index)
{
    index = index; // avoid compiler warning about unused variable
    return 0;
}

void Board::showArmedStatus(bool armed)
{
    // XXX this would be a good place to sound a buzzer!

    armed = armed; // avoid compiler warning about unused variable
}
 
void Board::showAuxStatus(uint8_t status)
{
    status = status; // avoid compiler warning about unused variable
}


bool Board::baroInit(void)
{
  return false;
}

void Board::baroUpdate(void)
{
}

int32_t Board::baroGetPressure(void)
{
    return 0;
}

void Board::checkReboot(bool pendReboot)
{
}


