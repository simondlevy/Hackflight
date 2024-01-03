/* 
   USFS interrupt example

   Copyright (C) 2022 TleraCorp && Simon D. Levy

   Adapted from

     https://github.com/kriswiner/USFS_SENtral_sensor_hub/tree/master/WarmStartandAccelCal

   This file is part of USFS.

   USFS is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   USFS is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with USFS.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <math.h>

#include <Wire.h>

#include "usfs.hpp"

// Set to 0 for polling version
static const uint8_t INTERRUPT_PIN = 12; 

static const uint8_t ACCEL_BANDWIDTH = 3;
static const uint8_t GYRO_BANDWIDTH  = 3;
static const uint8_t QUAT_DIVISOR    = 1;
static const uint8_t MAG_RATE        = 100;
static const uint8_t ACCEL_RATE      = 20; // Multiply by 10 to get actual rate
static const uint8_t GYRO_RATE       = 100; // Multiply by 10 to get actual rate
static const uint8_t BARO_RATE       = 50;

static const uint8_t INTERRUPT_ENABLE = Usfs::INTERRUPT_RESET_REQUIRED |
                                        Usfs::INTERRUPT_ERROR |
                                        Usfs::INTERRUPT_QUAT;

static const bool VERBOSE = true;

static const uint8_t REPORT_HZ = 2;

static volatile bool _gotNewData;

static void interruptHandler()
{
    _gotNewData = true;
}

static Usfs usfs;

void setup()
{
    Serial.begin(115200);
    delay(4000);

    Wire.begin(); 
    Wire.setClock(400000); 
    delay(1000);

    usfs.reportChipId();        

    usfs.loadFirmware(VERBOSE); 

    usfs.begin(
            ACCEL_BANDWIDTH,
            GYRO_BANDWIDTH,
            QUAT_DIVISOR,
            MAG_RATE,
            ACCEL_RATE,
            GYRO_RATE,
            BARO_RATE,
            INTERRUPT_ENABLE,
            VERBOSE); 

    if (INTERRUPT_PIN) {
        pinMode(INTERRUPT_PIN, INPUT);
        attachInterrupt(INTERRUPT_PIN, interruptHandler, RISING);  
    }

    // Clear interrupts
    Usfs::checkStatus();

    Serial.println("Enter '1' to proceed...");
    while (true) {
        if (Serial.read() == '1') {
            break;
        }
        delay(10);
    }

} // setup

void loop()
{
    static uint32_t _interruptCount;

    static float temperature, pressure;
    static float ax, ay, az;
    static float gx, gy, gz;
    static float mx, my, mz; 
    static float qw, qx, qy, qz;

    if ((INTERRUPT_PIN == 0) || _gotNewData) { 

        _gotNewData = false;  

        if (INTERRUPT_PIN) {
            _interruptCount++;
        }

        uint8_t eventStatus = Usfs::checkStatus(); 

        if (Usfs::eventStatusIsError(eventStatus)) { 

            Usfs::reportError(eventStatus);
        }

        if (Usfs::eventStatusIsAccelerometer(eventStatus)) { 

            usfs.readAccelerometerScaled(ax, ay, az);
        }

        if (Usfs::eventStatusIsGyrometer(eventStatus)) { 

            usfs.readGyrometerScaled(gx, gy, gz);
        }

        if (Usfs::eventStatusIsMagnetometer(eventStatus)) { 

            usfs.readMagnetometerScaled(mx, my, mz);
        }

        if (Usfs::eventStatusIsQuaternion(eventStatus)) { 
            usfs.readQuaternion(qw, qx, qy, qz);
        }

        if (Usfs::eventStatusIsBarometer(eventStatus)) { 
            pressure = usfs.readBarometerRaw() * 0.01f + 1013.25f; 
            temperature = usfs.readTemperatureRaw() * 0.01f; 
        }
    } 

    static uint32_t _msec;

    uint32_t msec = millis();

    static float yaw, pitch, roll;

    if (msec-_msec > 1000/REPORT_HZ) { 

        if (INTERRUPT_PIN) {
            Serial.print("Interrupts/sec: ");
            Serial.println(_interruptCount * REPORT_HZ);
        }

        _interruptCount = 0;
        _msec = msec;

        Serial.print("Ax = ");
        Serial.print((int)1000 * ax);
        Serial.print(" Ay = ");
        Serial.print((int)1000 * ay);
        Serial.print(" Az = ");
        Serial.print((int)1000 * az);
        Serial.println(" g");
        Serial.print("Gx = ");
        Serial.print( gx, 2);
        Serial.print(" Gy = ");
        Serial.print( gy, 2);
        Serial.print(" Gz = ");
        Serial.print( gz, 2);
        Serial.println(" deg/s");
        Serial.print("Mx = ");
        Serial.print( (int)mx);
        Serial.print(" My = ");
        Serial.print( (int)my);
        Serial.print(" Mz = ");
        Serial.print( (int)mz);
        Serial.println(" mG");

        Serial.println("Hardware quaternions:");
        Serial.print("Qw = ");
        Serial.print(qw);
        Serial.print(" Qx = ");
        Serial.print(qx);
        Serial.print(" Qy = ");
        Serial.print(qy);
        Serial.print(" Qz = ");
        Serial.println(qz);

        float A12 =   2.0f * (qx * qy + qw * qz);
        float A22 =   qw * qw + qx * qx - qy * qy - qz * qz;
        float A31 =   2.0f * (qw * qx + qy * qz);
        float A32 =   2.0f * (qx * qz - qw * qy);
        float A33 =   qw * qw - qx * qx - qy * qy + qz * qz;
        pitch = -asinf(A32);
        roll  = atan2f(A31, A33);
        yaw   = atan2f(A12, A22);
        pitch *= 180.0f / M_PI;
        yaw   *= 180.0f / M_PI;
        yaw   += 13.8f; 
        if (yaw < 0) yaw   += 360.0f ; 
        roll  *= 180.0f / M_PI;

        Serial.print("Hardware roll, pitch, yaw: ");
        Serial.print(roll, 2);
        Serial.print(", ");
        Serial.print(pitch, 2);
        Serial.print(", ");
        Serial.print(yaw, 2);
        Serial.println(" deg");

        Serial.print("Hardware ax, ay, az: ");
        Serial.print(ax * 1000, 2);
        Serial.print(", ");
        Serial.print(ay * 1000, 2);
        Serial.print(", ");
        Serial.print(az * 1000, 2);
        Serial.println(" g");

        Serial.println("MS5637:");
        Serial.print("Altimeter temperature = ");
        Serial.print(temperature, 2);
        Serial.println(" C"); 
        Serial.print("Altimeter pressure = ");
        Serial.print(pressure, 2);
        Serial.println(" mbar");
        Serial.print("Altitude = ");
        Serial.print(44307 * (1.0f - pow(((pressure) / 1013.25f), 0.190284f)));
        Serial.println(" m\n");
     } 

}  // loop
