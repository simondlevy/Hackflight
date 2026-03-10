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
#include <MPU6050.h>

// Hackflight library
#include <hackflight.h>
#include <firmware/datatypes.hpp>
#include <firmware/debugging.hpp>
#include <firmware/imu/old/imu.hpp>
#include <firmware/led.hpp>
#include <firmware/rx/dsmx.hpp>
#include <firmware/setpoint.hpp>
#include <firmware/timer.hpp>
#include <mixers/bfquadx.hpp>
#include <pidcontrol/stabilizer.hpp>

// Motors ---------------------------------------------------------

static DshotTeensy4 _motors = DshotTeensy4({6, 5, 4, 3});

// LED -------------------------------------------------------------

static hf::LED _led = hf::LED(13);

// PID control -----------------------------------------------------

static hf::StabilizerPid _stabilizerPid;

// Motor mixing ----------------------------------------------------

static hf::Mixer _mixer;

// FAFO -----------------------------------------------------------

static const uint32_t LOOP_FREQ_HZ = 2000;

// Main ----------------------------------------------------------------------

void setup()
{
    rx_init();

    Serial1.begin(115000);

    hf::initImu();

    delay(10);

    _motors.arm(); 

    _led.begin(); 
}

void loop()
{
    const auto usec_curr = micros();      

    const auto dt = hf::Timer::getDt();

    _led.blink(); 

    rx_read();

    const auto setpoint = hf::mksetpoint(rx_chanvals);

    const auto state = hf::getVehicleState(dt);

    hf::Debugger::debug(rx_is_armed, setpoint, state);
    //hf::Debugger::profile();

    _stabilizerPid = hf::StabilizerPid::run(_stabilizerPid,
            !rx_is_throttle_down, dt, state, setpoint);

    _mixer = hf::Mixer::run(_mixer, _stabilizerPid.setpoint);

    _motors.run(rx_is_armed, _mixer.motorvals);

    hf::Timer::runDelayLoop(usec_curr, LOOP_FREQ_HZ); 
}
