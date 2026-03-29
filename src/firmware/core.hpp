/*
   Hackflight main sketch

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

#pragma once

// Standard Arduino libraries
#include <Wire.h> 

// Third-party libraries
#include <Adafruit_VL53L1X.h>
#include <CRSFforArduino.hpp>
#include <dshot-teensy4.hpp>  

// Hackflight library
#include <hackflight.h>
#include <firmware/debugging.hpp>
#include <firmware/ekf/ekf.hpp>
#include <firmware/imu/imufilter.hpp>
#include <firmware/flying.hpp>
#include <firmware/led.hpp>
#include <firmware/rx/elrs.hpp>
#include <firmware/rxdata.hpp>
#include <firmware/safety.hpp>
#include <firmware/sensors/imu.hpp>
#include <firmware/sensors/zranger/zranger.h>
#include <firmware/setpoint.hpp>
#include <firmware/timer.hpp>
#include <mixers/bfquadx.hpp>
#include <firmware/profiling.hpp>
#include <pidcontrol/stabilizer.hpp>

/* Update rates
   MAIN_LOOP = 1000
   PIDS = 500;
 */

namespace hf {

    class Core {

        private:

            static constexpr float EKF_PREDICTION_RATE_HZ = 100;

            static constexpr float ZRANGER_ACQUISITION_RATE_HZ = 50;

        public:

            void setup(IMU & imu, RX & rx, DshotTeensy4 & motors, LED & led)
            {
                imu.begin();

                rx.begin();

                //ZRanger::begin();

                motors.begin(); 

                led.begin(); 
            }

            void loop(IMU & imu, RX & rx, DshotTeensy4 & motors, LED & led)
            {
                const auto loop_start_usec = micros();

                if (_zrangerTimer.ready(ZRANGER_ACQUISITION_RATE_HZ)) {
                }

                if (_ekfPredictionTimer.ready(EKF_PREDICTION_RATE_HZ)) {
                    _ekf = EKF::predict(_ekf, millis(), _flyingCheck.isFlying);
                }

                //static uint32_t _imucount; _imucount = Profiler::report(++_imucount);

                const auto dt = Timer::getDt();

                const auto rxdata =
                    _imuFilter.isGyroCalibrated ? rx.read() : RxData();

                const auto imuraw = imu.read();

                _imuFilter = ImuFilter::step(_imuFilter, millis(), imuraw,
                        imu.gyroRangeDps(), imu.accelRangeGs());

                led.blink(millis(), _imuFilter.isGyroCalibrated);

                _flyingCheck = _flyingCheck.run(
                        _flyingCheck, millis(), _mixer.motorvals, 4);

                _ekf = EKF::update(_ekf, millis(), _imuFilter.output);

                const auto state = EKF::getVehicleState(_ekf, _imuFilter.output);

                _mode = Safety::updateMode(state, rxdata, _imuFilter, _mode);

                const auto setpoint = mksetpoint(rxdata.axes);

                _stabilizerPid = StabilizerPid::run( _stabilizerPid,
                        !rxdata.is_throttle_down, dt, state, setpoint);

                _mixer = Mixer::run(_mixer, _stabilizerPid.setpoint);

                if (_mode != MODE_PANIC) {
                    motors.run(rxdata.is_armed, _mixer.motorvals);
                }

                //Debugger::report(state);

                Timer::runDelayLoop(loop_start_usec);

                //Profiler::report();
            }

        private:

            StabilizerPid _stabilizerPid;
            Mixer _mixer;
            ImuFilter _imuFilter;
            EKF _ekf;
            FlyingCheck _flyingCheck;
            mode_e _mode;
            Timer _ekfPredictionTimer;
            Timer _zrangerTimer;
    };
}
