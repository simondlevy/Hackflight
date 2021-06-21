/*
   Adafruit NXP implementation of softquare-quaternion IMU

   Copyright (c) 2020 Simon D. Levy

   MIT License
 */

#pragma once

#include "imus/softquat.hpp"

#include <Adafruit_Sensor.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>

namespace hf {

    class NxpSoftwareQuaternionIMU : public SoftwareQuaternionIMU {

        private:

            static const fxos8700AccelRange_t ACCEL_RANGE = ACCEL_RANGE_2G;

            Adafruit_FXOS8700   _accelmag = Adafruit_FXOS8700();
            Adafruit_FXAS21002C _gyro     = Adafruit_FXAS21002C();

            sensors_event_t _g_event;
            sensors_event_t _a_event;
            sensors_event_t _m_event;

            float mss2gs(float mss)
            {
                return mss / 9.8665;
            }

        protected:

            virtual void begin(void) override
            {
                _gyro.begin();
                _accelmag.begin(ACCEL_RANGE);
            }

            virtual bool imuReady(void) override 
            {
                return _gyro.getEvent(&_g_event) && _accelmag.getEvent(&_a_event, &_m_event);
            }

            virtual void imuReadAccelGyro(float & ax, float & ay, float & az, float & gx, float & gy, float &gz) override
            {
                ax = mss2gs(_a_event.acceleration.x);
                ay = mss2gs(_a_event.acceleration.y);
                az = mss2gs(_a_event.acceleration.z);

                gx = _g_event.gyro.x;
                gy = _g_event.gyro.y;
                gz = _g_event.gyro.z;
            }

    }; // class NxpSoftwareQuaternionIMU

} // namespace hf
