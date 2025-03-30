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

#include <BMI088.h>

//static Bmi088Accel accel(Wire, 0x18);
//static Bmi088Gyro gyro(Wire, 0x69);

// MOSI, MISO, SCLK
static SPIClass myspi = SPIClass (PB15, PB14, PB13);

// CS
static Bmi088Accel accel(myspi, PB1);
static Bmi088Gyro gyro(myspi, PB0);

void setup() 
{
    int status = 0;

    Serial.begin(115200);

    status = accel.begin();

    while (status < 0) {
        Serial.println("Accel Initialization Error");
        Serial.println(status);
        delay(500);
    }

    status = gyro.begin();
    while (status < 0) {
        Serial.println("Gyro Initialization Error");
        Serial.println(status);
        delay(500);
    }
}

void loop() 
{
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

    delay(20);
}
