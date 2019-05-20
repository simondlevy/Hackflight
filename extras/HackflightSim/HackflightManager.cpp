/*
   MulticopterSim FlightManager class implementation using Hackflight

   Copyright(C) 2019 Simon D.Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MEReceiverHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
   */

#include "FlightManager.h"
#include "SimBoard.hpp"
#include "SimSensors.hpp"

// Math support
#define _USE_MATH_DEFINES
#include <math.h>

#include <hackflight.hpp>
#include "SimReceiver.hpp"

// PID controllers
#include <pidcontrollers/level.hpp>
#include <pidcontrollers/althold.hpp>
#include <pidcontrollers/flowhold.hpp>

// Mixer
#include <mixers/quadxap.hpp>

// Debugging
#include "debug.h"
#include "osd.h"

class FHackflightManager : public FFlightManager {

    private:

        // PID tuning

        hf::Rate ratePid = hf::Rate(
                0.01,	// Roll/Pitch P
                0.01,	// Roll/Pitch I
                0.01,	// Roll/Pitch D
                0.5,	// Yaw P
                0.0,	// Yaw I
                8.f);	// Demands to rate


        hf::Level level = hf::Level(0.20f);

        hf::AltitudeHold althold = hf::AltitudeHold(
                1.00f,  // altHoldP
                0.50f,  // altHoldVelP
                0.01f,  // altHoldVelI
                0.10f); // altHoldVelD

        hf::FlowHold flowhold = hf::FlowHold(0.06);

        // Main firmware
        hf::Hackflight _hackflight;

        // Flight-controller board
        SimBoard _board;

        // "Receiver" (joystick/gamepad)
        SimReceiver _receiver;

        // Mixer
        hf::MixerQuadXAP _mixer;

        // "Sensors" (get values from dynamics)
        SimSensors * _sensors;

        // Gimbal axes
        float _gimbalRoll;
        float _gimbalPitch;

    public:

        FHackflightManager(double initialPosition[3], double initialRotation[3]) : 
            FFlightManager(4, initialPosition, initialRotation) // 4 motors
    {
        // Start Hackflight firmware, indicating already armed
        _hackflight.init(&_board, &_receiver, &_mixer, &ratePid, true);

        // Add simulated sensor suite
        _sensors = new SimSensors(_dynamics);
        _hackflight.addSensor(_sensors);

        // Add level PID controller for aux switch position 1
        _hackflight.addPidController(&level, 1);

        // Add altitude-hold and position-hold PID controllers in switch position 2
        _hackflight.addPidController(&althold, 2);    
        _hackflight.addPidController(&flowhold, 2);    

        // Start gimbal in center
        _gimbalRoll = 0;
        _gimbalPitch = 0;
    }

        virtual ~FHackflightManager(void)
        {
            delete _sensors;
        }

        virtual void getGimbal(float & roll, float &pitch) override
        {
            roll = _gimbalRoll;
            pitch = _gimbalPitch;
        }

        virtual void update(double time, double quat[4], double gyro[3], double * motorvals) override
        {
            _receiver.update();

            if (_receiver.inGimbalMode()) {
                _receiver.getGimbal(_gimbalRoll, _gimbalPitch);
            }

            _hackflight.update();

            // Input deltaT, quat, gyro; output motor values
            _board.update(time, quat, gyro, motorvals);
        }

}; // HackflightManager


static FFlightManager * _flightManager;

// Factory method for FlightManager class
FFlightManager * FFlightManager::createFlightManager(double initialPosition[3], double initialRotation[3])
{
    _flightManager = new FHackflightManager(initialPosition, initialRotation);
    return _flightManager;
}

// Asynchronous debugging
void hf::Board::outbuf(char * buf)
{
    _flightManager->dbgprintf("%s", buf);
}
