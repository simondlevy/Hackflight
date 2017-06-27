/* 
   altitude.hpp: Altitude estimator

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

#include "filter.hpp"
#include "config.hpp"
#include "barometer.hpp"
#include "accelerometer.hpp"

namespace hf {
 
class Altitude {

    public:

        void     init(const AltitudeConfig & _config);
        void     computePid(float baroPressure, float eulerAnglesDegrees[3], uint32_t currentTimeUsec);
        void     handleAuxSwitch(uint8_t auxState, uint16_t throttleDemand);
        void     updateAccel(int16_t accelRaw[3], float eulerAnglesRadians[3], uint32_t currentTimeUsec, bool armed);
        uint16_t modifyThrottleDemand(uint16_t throttleDemand);

    private:

        AltitudeConfig config;

        // Barometer
        Barometer baro;
        uint32_t  baroCalibrationStart;
        int32_t   lastBaroAlt;

        // IMU
        Accelerometer accel;

        // Fused
        int32_t  altHold;
        int32_t  altPid;
        int32_t  errorVelocityI;
        int32_t  estimAlt;
        bool     holdingAltitude;
        int16_t  initialThrottleHold;
        bool     isAltHoldChanged;
        int32_t  setVelocity;      
        bool     velocityControl; 
};

/********************************************* CPP ********************************************************/

void Altitude::init(const AltitudeConfig & _config)
{
    memcpy(&config, &_config, sizeof(AltitudeConfig));

    baro.init(config.baro);
    baroCalibrationStart = 0;
    lastBaroAlt = 0;

    accel.init(config.accel);

    initialThrottleHold = 0;
    setVelocity = 0;      
    velocityControl = false; 
    altPid = 0;
    errorVelocityI = 0;
    holdingAltitude = false;
    isAltHoldChanged = false;
}


void Altitude::updateAccel(int16_t accelRaw[3], float eulerAnglesRadians[3], uint32_t currentTimeUsec, bool armed)
{
    accel.update(accelRaw, eulerAnglesRadians, currentTimeUsec, armed);

}

uint16_t Altitude::modifyThrottleDemand(uint16_t throttleDemand)
{
    if (holdingAltitude) {
        if (config.fastChange) {
            // rapid alt changes
            if (abs(throttleDemand - initialThrottleHold) > config.throttleNeutral) {
                errorVelocityI = 0;
                isAltHoldChanged = true;
                throttleDemand += (throttleDemand > initialThrottleHold) ? -config.throttleNeutral : config.throttleNeutral;
            } else {
                if (isAltHoldChanged) {
                    altHold = estimAlt;
                    isAltHoldChanged = false;
                }
                throttleDemand = constrain(initialThrottleHold + altPid, config.throttleMin, config.throttleMax);
            }
        } else {
            // slow alt changes
            if (abs(throttleDemand - initialThrottleHold) > config.throttleNeutral) {
                // set velocity proportional to stick movement +100 throttle gives ~ +50 cm/s
                setVelocity = (throttleDemand - initialThrottleHold) / 2;
                velocityControl = true;
                isAltHoldChanged = true;
            } else if (isAltHoldChanged) {
                altHold = estimAlt;
                velocityControl = false;
                isAltHoldChanged = false;
            }
            throttleDemand = constrain(initialThrottleHold + altPid, config.throttleMin, config.throttleMax);
        }
    }

    return throttleDemand;

} // modifyThrottleDemand

void Altitude::computePid(float baroPressure, float eulerAnglesDegrees[3], uint32_t currentTimeUsec)
{  
    static uint32_t previousTimeUsec;
    uint32_t dT_usec = currentTimeUsec - previousTimeUsec;
    previousTimeUsec = currentTimeUsec;

    // Start baro calibration if not yet started
    if (!baroCalibrationStart) 
        baroCalibrationStart = currentTimeUsec;

    // Update the baro with the current pressure reading
    baro.update(baroPressure);

    // Reset velocity and acceleration during baro calibration
    if (currentTimeUsec - baroCalibrationStart < 1e6*config.baro.calibrationSeconds) {
        baro.calibrate();
        accel.reset();
    }

    // Get estimated altitude from baro
    int32_t baroAltitude = baro.getAltitude();

    // Integrate accelerometer to get altitude and velocity
    accel.integrate();

    // Apply complementary filter to fuse baro and accel altitude
    estimAlt = Filter::complementary(accel.getAltitude(), baroAltitude, config.cfAlt);

    // Compute velocity from barometer
    int32_t baroVelocity = (baroAltitude - lastBaroAlt) * 1000000.0f / dT_usec;
    lastBaroAlt = baroAltitude;
    baroVelocity = constrain(baroVelocity, -1500, 1500);    // constrain baro velocity +/- 1500cm/s
    baroVelocity = Filter::deadband(baroVelocity, 10);         // to reduce noise near zero

    // Apply complementary filter to keep the calculated velocity based on baro velocity (i.e. near real velocity).
    // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e without delay
    float fusedVelocity = Filter::complementary(accel.getVelocity(), baroVelocity, config.cfVel);
    accel.adjustVelocity(fusedVelocity);

    // only calculate pid if the copters thrust is facing downwards(<80deg)
    altPid = 0;
    float tiltAngle = Filter::max(abs(eulerAnglesDegrees[0]), abs(eulerAnglesDegrees[1]));

    if (tiltAngle < config.maxTiltAngle) { 

        int32_t setVel = setVelocity;
        int32_t error = 0;

        // Altitude P-Controller
        if (!velocityControl) {
            error = constrain(altHold - estimAlt, -500, 500);
            error = Filter::deadband(error, 10);       // remove small P parametr to reduce noise near zero position
            setVel = constrain(config.pidAltP * error, -300, +300); // limit velocity to +/- 3 m/s
        } 

        // Velocity PID-Controller
        // P
        error = setVel - (int32_t)lrintf(fusedVelocity);
        altPid = constrain(config.pidVelP * error, -300, +300);

        // I
        errorVelocityI += (config.pidVelI * error);
        errorVelocityI = constrain(errorVelocityI, -200, +200);
        altPid += errorVelocityI;

        // D
        altPid -= constrain(config.pidVelD * accel.getAcceleration(), -150, 150);
    } 

} // computePid

void Altitude::handleAuxSwitch(uint8_t auxState, uint16_t throttleDemand)
{
    if (auxState > 0) {
        holdingAltitude = true;
        initialThrottleHold = throttleDemand;
        altHold = estimAlt;
        errorVelocityI = 0;
        altPid = 0;
    }
    else {
        holdingAltitude = false;
    }
}

} // namespace hf
