/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under the
   terms of the GNU General Public License as published by the Free Software
   Foundation, either version 3 of the License, or (at your option) any later
   version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
   WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
   PARTICULAR PURPOSE. See the GNU General Public License for more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <usfs.hpp>

#include "axes.h"
#include "constrain.h"
#include "filters/pt1.h"
#include "pid.h"
#include "utils.h"
#include "datatypes.h"

class Imu {

    public:

        Imu(void)
        {
            _gyroScale = GYRO_SCALE_DPS / 32768.;
        }

        static float deg2rad(float deg)
        {
            return deg * M_PI / 180;
        }

        static void getEulerAngles(const vehicleState_t & state, int16_t angles[3])
        {
            angles[0] = (int16_t)(10 * rad2degi(state.phi));
            angles[1] = (int16_t)(10 * rad2degi(state.theta));
            angles[2] = (int16_t)rad2degi(state.psi);
        }

        float qw;
        float qx;
        float qy;
        float qz;

        void begin(void)
        {
            setGyroCalibrationCycles();
        }

        Axes getEulerAngles(const uint32_t time)
        {
            (void)time;

            Axes angles = quat2euler(qw, qx, qy, qz);

            return Axes(angles.x, -angles.y, -angles.z);
        }

        bool gyroIsCalibrating(void)
        {
            return _gyroIsCalibrating;
        }

        void gyroRawToFilteredDps(int16_t rawGyro[3], vehicleState_t & state)
        {
            accumulateGyro(_gyroX.dpsFiltered, _gyroY.dpsFiltered, _gyroZ.dpsFiltered);

            const auto calibrationComplete = _gyroCalibrationCyclesRemaining <= 0;

            static Axes _adc;

            if (calibrationComplete) {

                // move 16-bit gyro data into floats to avoid overflows in
                // calculations

                _adc.x = readCalibratedGyro(rawGyro, _gyroX, 0);
                _adc.y = readCalibratedGyro(rawGyro, _gyroY, 1);
                _adc.z = readCalibratedGyro(rawGyro, _gyroZ, 2);

                scaleGyro(_gyroX, _adc.x);
                scaleGyro(_gyroY, _adc.y);
                scaleGyro(_gyroZ, _adc.z);
            } 

            else {
                calibrateGyro(rawGyro);
            }

            // Use gyro lowpass 2 filter for downsampling
            applyGyroLpf2(_gyroX);
            applyGyroLpf2(_gyroY);
            applyGyroLpf2(_gyroZ);

            // Then apply lowpass 1
            applyGyroLpf1(_gyroX);
            applyGyroLpf1(_gyroY);
            applyGyroLpf1(_gyroZ);

            _gyroIsCalibrating = !calibrationComplete;

            state.dphi   = _gyroX.dpsFiltered; 
            state.dtheta = _gyroY.dpsFiltered; 
            state.dpsi   = _gyroZ.dpsFiltered;
        }


        void updateAccelerometer(const int16_t rawAccel[3])
        {
            (void)rawAccel;
        }

        int32_t getGyroSkew(
                const uint32_t nextTargetCycles,
                const int32_t desiredPeriodCycles)
        {
            const auto skew =
                intcmp(nextTargetCycles, _gyroSyncTime) % desiredPeriodCycles;

            return skew > (desiredPeriodCycles / 2) ? skew - desiredPeriodCycles : skew;
        }

    private:

        static const uint32_t GYRO_CALIBRATION_DURATION      = 1250000;
        static const uint16_t GYRO_LPF1_DYN_MIN_HZ           = 250;
        static const uint16_t GYRO_LPF2_STATIC_HZ            = 500;
        static const uint8_t  MOVEMENT_CALIBRATION_THRESHOLD = 48;

        static const uint16_t GYRO_SCALE_DPS  = 2000;

        Usfs usfs;

        static float rad2deg(float rad)
        {
            return 180 * rad / M_PI;
        }

        static int16_t rad2degi(float rad)
        {
            return (int16_t)rad2deg(rad);
        }

        class Stats {

            private:

                float  _oldM;
                float  _newM;
                float  _oldS;
                float  _newS;
                int32_t _n;

                float variance(void)
                {
                    return ((_n > 1) ? _newS / (_n - 1) : 0.0f);
                }

            public:

                void stdevClear(void)
                {
                    _n = 0;
                }

                void stdevPush(float x)
                {
                    _n++;

                    if (_n == 1) {
                        _oldM = _newM = x;
                        _oldS = 0.0f;
                    } else {
                        _newM = _oldM + (x - _oldM) / _n;
                        _newS = _oldS + (x - _oldM) * (x - _newM);
                        _oldM = _newM;
                        _oldS = _newS;
                    }
                }

                float stdevCompute(void)
                {
                    return sqrtf(variance());
                }
        }; 

        typedef struct {
            float sum[3];
            Stats stats[3];
        } calibration_t;

        typedef struct {

            float dps;           // aligned, calibrated, scaled, unfiltered
            float dpsFiltered;   // filtered 
            float sampleSum;     // summed samples used for downsampling
            float zero;

            Pt1Filter lowpassFilter1 = Pt1Filter(GYRO_LPF1_DYN_MIN_HZ);
            Pt1Filter lowpassFilter2 = Pt1Filter(GYRO_LPF2_STATIC_HZ);

        } gyroAxis_t;

        calibration_t _gyroCalibration;

        int32_t  _gyroCalibrationCyclesRemaining;
        float    _gyroScale;
        bool     _gyroIsCalibrating;

        gyroAxis_t _gyroX;
        gyroAxis_t _gyroY;
        gyroAxis_t _gyroZ;

        static uint32_t calculateGyroCalibratingCycles(void)
        {
            return GYRO_CALIBRATION_DURATION / PidController::PERIOD;
        }

        void calibrateGyroAxis(int16_t rawGyro[3], gyroAxis_t & axis, const uint8_t index)
        {
            // Reset at start of calibration
            if (_gyroCalibrationCyclesRemaining ==
                    (int32_t)calculateGyroCalibratingCycles()) {
                _gyroCalibration.sum[index] = 0.0f;
                _gyroCalibration.stats[index].stdevClear();
                // zero is set to zero until calibration complete
                axis.zero = 0.0f;
            }

            // Sum up CALIBRATING_GYRO_TIME_US readings
            _gyroCalibration.sum[index] += rawGyro[index];
            _gyroCalibration.stats[index].stdevPush(rawGyro[index]);

            if (_gyroCalibrationCyclesRemaining == 1) {
                const float stddev = _gyroCalibration.stats[index].stdevCompute();

                // check deviation and startover in case the model was moved
                if (MOVEMENT_CALIBRATION_THRESHOLD && stddev >
                        MOVEMENT_CALIBRATION_THRESHOLD) {
                    setGyroCalibrationCycles();
                    return;
                }

                axis.zero = _gyroCalibration.sum[index] / calculateGyroCalibratingCycles();
            }
        }

        void calibrateGyro(int16_t rawGyro[3])
        {
            calibrateGyroAxis(rawGyro, _gyroX, 0);
            calibrateGyroAxis(rawGyro, _gyroY, 1);
            calibrateGyroAxis(rawGyro, _gyroZ, 2);

            --_gyroCalibrationCyclesRemaining;
        }

        void applyGyroLpf1(gyroAxis_t & axis)
        {
            axis.dpsFiltered = axis.lowpassFilter1.apply(axis.sampleSum);
        }

        void applyGyroLpf2(gyroAxis_t & axis)
        {
            axis.sampleSum = axis.lowpassFilter2.apply(axis.dps);
        }

        void scaleGyro(gyroAxis_t & axis, const float adc)
        {
            axis.dps = adc * _gyroScale; 
        }

        float readCalibratedGyro(int16_t rawGyro[3], gyroAxis_t & axis, uint8_t index)
        {
            return rawGyro[index] - axis.zero;
        }

        typedef Axes (*rotateFun_t)(Axes & axes);

        uint32_t _gyroSyncTime;

        void setGyroCalibrationCycles(void)
        {
            _gyroCalibrationCyclesRemaining = (int32_t)calculateGyroCalibratingCycles();
        }

        static auto quat2euler(
                const float qw, const float qx, const float qy, const float qz) -> Axes 
        {
            const auto phi = atan2(2.0f*(qw*qx+qy*qz), qw*qw-qx*qx-qy*qy+qz*qz);
            const auto theta = asin(2.0f*(qx*qz-qw*qy));
            const auto psi = atan2(2.0f*(qx*qy+qw*qz), qw*qw+qx*qx-qy*qy-qz*qz);

            // Convert heading from [-pi,+pi] to [0,2*pi]
            return Axes(phi, theta, psi + (psi < 0 ? 2*M_PI : 0)); 
        }

        // For software quaternion
        void accumulateGyro(float x, float y, float z)
        {
            (void)x;
            (void)y;
            (void)z;
        }

}; // class Imu
