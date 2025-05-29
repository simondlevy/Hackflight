/*
 * Brian R Taylor
 * brian.taylor@bolderflight.com
 * 
 * Copyright (c) 2018 Bolder Flight Systems
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
 * and associated documentation files (the "Software"), to deal in the Software without restriction, 
 * including without limitation the rights to use, copy, modify, merge, publish, distribute, 
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or 
 * substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "BMI088.h"

//static const uint8_t ACCEL_INTERRUPT_PIN = 3;
static const uint8_t GYRO_INTERRUPT_PIN = 4;

static Bmi088Accel accel(Wire,0x19);

static Bmi088Gyro gyro(Wire,0x69);

static volatile bool accel_flag, gyro_flag;

//static uint32_t accel_count, gyro_count;

/*
void accel_drdy()
{
    accel_flag = true;
    accel_count++;
}
*/

void gyro_drdy()
{
    gyro_flag = true;
    //gyro_count++;
}

void setup() 
{
    int status = 0;

    Serial.begin(115200);
    while(!Serial) {}

    status = accel.begin();
    if (status < 0) {
        Serial.println("Accel Initialization Error");
        Serial.println(status);
        while (1) {}
    }
    status = accel.setOdr(Bmi088Accel::ODR_12_5HZ_BW_1HZ/*ODR_100HZ_BW_19HZ*/);
    //status = accel.pinModeInt1(Bmi088Accel::PUSH_PULL,Bmi088Accel::ACTIVE_HIGH);
    //status = accel.mapDrdyInt1(true);

    status = gyro.begin();
    if (status < 0) {
        Serial.println("Gyro Initialization Error");
        Serial.println(status);
        while (1) {}
    }

    status = gyro.setOdr(Bmi088Gyro::ODR_100HZ_BW_12HZ);
    status = gyro.pinModeInt3(Bmi088Gyro::PUSH_PULL,Bmi088Gyro::ACTIVE_HIGH);
    status = gyro.mapDrdyInt3(true);

    //pinMode(3,INPUT);
    //attachInterrupt(ACCEL_INTERRUPT_PIN,accel_drdy,RISING);

    pinMode(4,INPUT);
    attachInterrupt(GYRO_INTERRUPT_PIN,gyro_drdy,RISING);  
}

void loop() 
{
    //printf("accel:%lu  gyro:%lu\n", accel_count, gyro_count);

    if (/*accel_flag &&*/ gyro_flag) {

        //accel_flag = false;
        gyro_flag = false;

        accel.readSensor();
        gyro.readSensor();

        Serial.print(accel.getAccelX_mss());
        Serial.print("\t");
        Serial.print(accel.getAccelY_mss());
        Serial.print("\t");
        Serial.print(accel.getAccelZ_mss());
        Serial.print("\t");
        Serial.print(gyro.getGyroX_rads());
        Serial.print("\t");
        Serial.print(gyro.getGyroY_rads());
        Serial.print("\t");
        Serial.print(gyro.getGyroZ_rads());
        Serial.print("\t");
        Serial.print(accel.getTemperature_C());
        Serial.print("\n");
    }
}
