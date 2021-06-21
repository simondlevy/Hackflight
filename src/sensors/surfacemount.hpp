/*
   Abstract class for surface-mounted sensors (IMU, barometer)

   Copyright (c) 2018 Simon D. Levy
   
   MIT License
 */

#pragma once

#include "sensor.hpp"
#include "imu.hpp"

namespace hf {

    class SurfaceMountSensor : public Sensor {

        friend class Hackflight;

        protected:

            IMU * imu = NULL;

    };  // class SurfaceMountSensor

} // namespace
