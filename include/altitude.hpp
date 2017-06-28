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

        void init(const AltitudeConfig & _config, Board * _board);
        void handleAuxSwitch(uint8_t auxState, uint16_t throttleDemand);
        void computePid(bool armed);
        void updateAccelerometer(float eulerAnglesRadians[3], bool armed);
        void modifyThrottleDemand(int16_t & throttleDemand);

    private:

        AltitudeConfig config;

        Board * board;

        // Barometer
        Barometer baro;
        uint32_t  baroCalibrationStart;
        int32_t   lastBaroAlt;

        // IMU
        Accelerometer accel;

        // Fused
        int32_t  AltHold;
        int32_t  BaroPID;
        int32_t  EstAlt;
        bool     holdingAltitude;
        int16_t  initialThrottleHold;
};

/********************************************* CPP ********************************************************/

void Altitude::init(const AltitudeConfig & _config, Board * _board)
{
    memcpy(&config, &_config, sizeof(AltitudeConfig));

    board = _board;

    baro.init(config.baro);
    baroCalibrationStart = 0;
    lastBaroAlt = 0;

    accel.init(config.accel);

    initialThrottleHold = 0;
    BaroPID = 0;
    holdingAltitude = false;
}

void Altitude::handleAuxSwitch(uint8_t auxState, uint16_t throttleDemand)
{
    if (auxState > 0) {
        holdingAltitude = true;
        initialThrottleHold = throttleDemand;
        AltHold = EstAlt;
        BaroPID = 0;
    }
    else {
        holdingAltitude = false;
    }
}

void Altitude::modifyThrottleDemand(int16_t & throttleDemand)
{
    if (holdingAltitude) {

        throttleDemand = constrain(initialThrottleHold + BaroPID, config.throttleMin, config.throttleMax);
    }
}

void Altitude::computePid(bool armed)
{  
    // Update the baro with the current pressure reading
    baro.update(board->extrasGetBaroPressure());

    // Calibrate baro while not armed
    if (!armed) {
        baro.calibrate();
        return;
    }

    // Get estimated altitude from baro
    int32_t BaroAlt = baro.getAltitude();

    EstAlt = BaroAlt;

    // Apply deadband filter to accelerometer Z
    //float accZ = Filter::deadband(accel.accZ, config.accel.deadband);

    //P
    int16_t error16 = constrain(AltHold - EstAlt, -300, 300);
    error16 = Filter::deadband(error16, 10); //remove small P parametr to reduce noise near zero position
    BaroPID = constrain((config.pidP * error16 >>7), -150, +150);

    Serial.println(BaroPID);

    /*
    //I
    errorAltitudeI += conf.pid[PIDALT].I8 * error16 >>6;
    errorAltitudeI = constrain(errorAltitudeI,-30000,30000);
    BaroPID += errorAltitudeI>>9; //I in range +/-60
 
    applyDeadband(accZ, ACC_Z_DEADBAND);

    static int32_t lastBaroAlt;
    // could only overflow with a difference of 320m, which is highly improbable here
    int16_t baroVel = mul((EstAlt - lastBaroAlt) , (1000000 / UPDATE_INTERVAL));

    lastBaroAlt = EstAlt;

    baroVel = constrain(baroVel, -300, 300); // constrain baro velocity +/- 300cm/s
    applyDeadband(baroVel, 10); // to reduce noise near zero

    // Integrator - velocity, cm/sec
    vel += accZ * ACC_VelScale * dTime;

    // apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity). 
    // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e without delay
    vel = vel * 0.985f + baroVel * 0.015f;

    //D
    vario = vel;
    applyDeadband(vario, 5);
    BaroPID -= constrain(conf.pid[PIDALT].D8 * vario >>4, -150, 150);
    */
 
} // computePid

void Altitude::updateAccelerometer(float eulerAnglesRadians[3], bool armed)
{
    // Throttle modification is synched to aquisition of new IMU data
    int16_t accelRaw[3];
    board->extrasImuGetAccel(accelRaw);
    accel.update(accelRaw, eulerAnglesRadians, board->getMicros(), armed);
}

} // namespace hf
