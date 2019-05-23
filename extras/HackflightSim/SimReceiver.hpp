/*
   Hackflight Receiver subclass for MulticopterSim Allows us to treat an input
   device (joystick, game controller, R/C transmitter) as a "virtual receiver"
   for the firmware.

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

#pragma once

#include <receiver.hpp>

#include "joystick/Joystick.h"

class SimReceiver : public hf::Receiver {

    friend class FHackflightManager;

    public:

    SimReceiver(uint16_t updateFrequency=50)
    {
        _joystick = new Joystick();

        _deltaT = 1./updateFrequency;
        _previousTime = 0;
        _buttonState = 0;
    }

    void begin(void)
    {
    }

    bool gotNewFrame(void)
    {
        // Get a high-fidelity current time value from the OS
        double currentTime = FPlatformTime::Seconds();

        if (currentTime-_previousTime > _deltaT) {
            _previousTime = currentTime;
            return true;
        }

        return false;
    }

    void readRawvals(void)
    {
        // For game controllers, use buttons to fake up values in a three-position aux switch
        if (!_joystick->isRcTransmitter()) {
            rawvals[4] = buttonsToAux[_buttonState];
        }
    }

    Joystick::error_t update(void)
    {
        // Joystick::poll() returns zero (okay) or a postive value (error)
        Joystick::error_t pollResult = _joystick->poll(rawvals, _buttonState);

        // In gimbal mode, grab pan,tilt from cyclic stick, then lock roll and pitch at zero
        if (!pollResult && _joystick->inGimbalMode()) {
            _gimbalRoll  = rawvals[1];
            _gimbalPitch = rawvals[2];
            rawvals[1] = 0;
            rawvals[2] = 0;
        }

        return pollResult;
    }

    bool inGimbalMode(void)
    {
        return _joystick->inGimbalMode();
    }

    void getGimbal(float & roll, float & pitch)
    {
        roll  = _gimbalRoll;
        pitch = _gimbalPitch;
    }

    protected:

    uint8_t getAux1State(void) 
    {
        return Receiver::getAux1State();
    }

    uint8_t getAux2State(void)
    {
        // Always armed!
        return 1;
    }


    // Simulate auxiliary switch via pushbuttons
    uint8_t _buttonState;
    const float buttonsToAux[3] = {-.1f, 0.f, .8f};


    private:

    Joystick * _joystick;

    // Helps mock up periodic availability of new data frame (output data rate; ODR)
    double _deltaT;
    double _previousTime;

    float _gimbalRoll;
    float _gimbalPitch;

}; // class SimReceiver
