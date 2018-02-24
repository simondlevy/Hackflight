/* 
    altitude.hpp: Altitude estimation via barometer/accelerometer fusion

    Adapted from

    https://github.com/multiwii/baseflight/blob/master/src/imu.c

    This file is part of Hackflight.

    Hackflight is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Hackflight is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with EM7180.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "filter.hpp"
#include "barometer.hpp"
#include "imu.hpp"
#include "debug.hpp"
#include "datatypes.hpp"


namespace hf {

    class AltitudeEstimator {

        private: 

            // XXX for debugging
            static void printgauge(float x)
            {
                char c = x<0 ? '-' : '+';
                for (uint8_t k=0; k<abs(x); ++k) {
                    printf("%c", c);
                }
                printf("\n");
            }

            // Complementry filter for accel/baro
            const float    cfAlt  = 0.965f;
            const float    cfVel  = 0.985f;

            // Barometer
            Barometer baro;

            // IMU
            IMU imu;

            // fused
            float fusedAlt;
            float fusedVel;

            // PIDS: XXX Use uint8_t for now; eventually will be float
            uint8_t altP;
            uint8_t velP;
            uint8_t velI;
            uint8_t velD;

            // State variables
            float altHold;
            bool holding;
            float initialThrottleHold;  // [0,1]  
            float pid;
            float errorVelocityI;
            float accZ_old;

            // No velocity control for now
            bool velocityControl = false;

        public:

            AltitudeEstimator(uint8_t _altP, uint8_t _velP, uint8_t _velI, uint8_t _velD) 
            {
                altP = _altP;  
                velP = _velP;  
                velI = _velI;  
                velD = _velD; 
            }

            void init(void)
            {
                baro.init();
                imu.init();
                initialThrottleHold = 0;
                holding = false;
                pid = 0;
                errorVelocityI = 0;
                accZ_old = 0;
            }

            void handleAuxSwitch(demands_t & demands)
            {
                // Start
                if (demands.aux > 0) {
                    holding = true;
                    initialThrottleHold = demands.throttle;
                    altHold = fusedAlt;
                    errorVelocityI = 0;
                    accZ_old = 0;
                }

                // Stop
                else {
                    holding = false;
                }
            }

            void updateAccel(float accel[3], uint32_t currentTime)
            {
                imu.updateAccel(accel, currentTime);
            }

            void updateGyro(float gyro[3], uint32_t currentTime)
            {
                imu.updateGyro(gyro, currentTime);
            }

            void updateBaro(bool armed, float pressure, uint32_t currentTime)
            {  
                // Send pressure to baro for altitude estimation
                baro.update(pressure);

                // Calibrate baro AGL at rest
                if (!armed) {
                    baro.calibrate();
                    fusedAlt = 0;
                    fusedVel = 0;
                    return;
                }

                // Track delta time in seconds
                static uint32_t previousTime;
                float dt = (currentTime-previousTime) / 1.e6;
                previousTime = currentTime;

                // Get estimated altitude from barometer
                float baroAlt = baro.getAltitude();

                // Apply complementary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity). 
                // By using CF it's possible to correct the drift of integrated accelerometer velocity without loosing the phase, 
                // i.e without delay.
                float imuVel = imu.getVerticalVelocity();

                // Integrator - Altitude in cm
                fusedAlt += (imuVel * 0.5f) * dt + fusedVel * dt;   
                fusedAlt = Filter::complementary(fusedAlt, baroAlt, cfAlt);      

                fusedVel += imuVel;

                float baroVel = baro.getVelocity(currentTime);

                fusedVel = Filter::complementary(fusedVel, baroVel, cfVel);

                float accZ_tmp = imu.getVerticalAcceleration();

                if (holding) {

                    int32_t setVel = 0;
                    int32_t error = 0;

                    // Altitude P-Controller
                    if (!velocityControl) {
                        error = Filter::constrainAbs(altHold - fusedAlt, 500);
                        error = Filter::deadband(error, 10);       // remove small P parametr to reduce noise near zero position
                        setVel = Filter::constrainAbs((altP * error / 128), 300); // limit velocity to +/- 3 m/s
                    } else {
                        //setVel = setVelocity;
                    }

                    // Velocity PID-Controller
                    // P
                    error = setVel - fusedVel;
                    pid = Filter::constrainAbs((velP * error / 32), 300);

                    // I
                    errorVelocityI += (velI * error);
                    errorVelocityI = Filter::constrainAbs(errorVelocityI, 8196 * 200);
                    pid += errorVelocityI / 8196;     // I in the range of +/-200

                    // D
                    pid -= Filter::constrainAbs(velD * (accZ_tmp + accZ_old) / 512, 150);
                    
                    pid /= 500; // scale down to [-1,+1]
                }

                accZ_old = accZ_tmp;

            } // estimate

            void modifyDemands(demands_t & demands)
            {
                if (holding) {

                    demands.throttle = initialThrottleHold+pid;
                }
            }

    }; // class AltitudeEstimator

} // namespace hf
