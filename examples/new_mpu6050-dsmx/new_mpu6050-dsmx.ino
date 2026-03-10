/*
   Hackflight for Teensy 4.0 with DSMX receiver

   Based on  https://github.com/nickrehm/dRehmFlight

   Copyright (C) 2026 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

// Standard Arduino libraries
#include <Wire.h> 

// Third-party libraries
#include <dshot-teensy4.hpp>  
#include <dsmrx.hpp>  

// Hackflight library
#include <hackflight.h>
#include <firmware/datatypes.hpp>
#include <firmware/debugging.hpp>
#include <firmware/estimators/madgwick/new_madgwick.hpp>
#include <firmware/imu/mpu6050.hpp>
#include <firmware/imu/oldfilter.hpp>
#include <firmware/led.hpp>
#include <firmware/rx/dsmx.hpp>
#include <firmware/setpoint.hpp>
#include <firmware/timer.hpp>
#include <mixers/bfquadx.hpp>
#include <pidcontrol/stabilizer.hpp>

static DshotTeensy4 _motors = DshotTeensy4({6, 5, 4, 3});

static hf::LED _led = hf::LED(13);

static hf::IMU _imu;

static hf::ImuFilter _imuFilter;

static hf::MadgwickFilter _madgwick;

static hf::StabilizerPid _stabilizerPid;

static hf::Mixer _mixer;

static const uint32_t LOOP_FREQ_HZ = 2000;

void setup()
{
    rx_init();

    Serial1.begin(115000);

    _imu.begin();

    delay(10);

    _motors.arm(); 

    _led.begin(); 
}

void loop()
{
    const auto loop_start_usec = micros();      

    _led.blink(); 

    rx_read();

    const auto setpoint = hf::mksetpoint(rx_chanvals);

    const auto imuraw = _imu.read();

    const auto imufilt = _imuFilter.run(imuraw);

    const auto dt = hf::Timer::getDt();

    _madgwick = hf::MadgwickFilter::run(_madgwick, dt, imufilt);

    hf::Debugger::debug(_madgwick.state);
    //hf::Debugger::profile();

    _stabilizerPid = hf::StabilizerPid::run(_stabilizerPid,
            !rx_is_throttle_down, dt, _madgwick.state, setpoint);

    _mixer = hf::Mixer::run(_mixer, _stabilizerPid.setpoint);

    _motors.run(rx_is_armed, _mixer.motorvals);

    hf::Timer::runDelayLoop(loop_start_usec, LOOP_FREQ_HZ); 
}
