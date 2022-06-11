#include <Arduino.h>
#include <Wire.h>
#include <USFSMAX.h>

#include <imu.h>

// Geomagnetic field data for Lexington, Virginia on 8 June 2022.
// https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm
// Units are uT, angles are in decimal degrees.
static const float M_V             = 45.5967;
static const float M_H             = 21.5787;
static const float MAG_DECLINATION = -9.17417;

static const uint32_t I2C_CLOCK = 1000000;

// Interrupt pin
static const uint8_t INT_PIN = 10;

static USFSMAX usfsmax;

// Host DRDY interrupt handler
static volatile bool dataReady = true;
static void handleInterrupt()
{
    dataReady = true;
}

void imuGetQuaternion(hackflight_t * hf, uint32_t time, quaternion_t * quat)
{
    (void)hf;
    (void)time;
    (void)quat;
}

void imuInit(void)
{
    usfsmax.setGeoMag(M_V, M_H, MAG_DECLINATION);

    Wire.setClock(100000);      

    delay(100);

    usfsmax.begin();

    Wire.setClock(I2C_CLOCK); 

    delay(100);

    pinMode(INT_PIN, INPUT);

    attachInterrupt(INT_PIN, handleInterrupt, RISING);        

}

void imuUpdateFusion(hackflight_t * hf, uint32_t time, quaternion_t * quat, rotation_t * rot)
{
    (void)hf;
    (void)time;
    (void)quat;
    (void)rot;
}
