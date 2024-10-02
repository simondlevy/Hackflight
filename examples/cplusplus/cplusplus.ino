/*
  Hackflight example sketch custom QuadX frame with Spektrum DSMX receiver

  Adapted from https://github.com/nickrehm/dRehmFlight
 
  Copyright (C) 2024 Simon D. Levy
 
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

#include <turtle_board.hpp>

#include <pids/pitch_roll_angle.hpp>
#include <pids/pitch_roll_rate.hpp>
#include <pids/yaw_rate.hpp>

static hf::TurtleBoard _board;

// PID control ---------------------------------------------------------------

static constexpr float THROTTLE_DOWN = 0.06;

static hf::YawRatePid _yawRatePid;

static hf::PitchRollAnglePid _pitchRollAnglePid;
static hf::PitchRollRatePid _pitchRollRatePid;


void setup() 
{
    _board.init();
}

void loop() 
{
    uint32_t usec_curr=0;
    float dt=0;
    float thrustDemand=0, rollDemand=0, pitchDemand=0, yawDemand=0;
    float phi=0, theta=0, psi=0, gyroX=0, gyroY=0, gyroZ=0;

    static uint32_t _chan_1, _chan_2, _chan_3, _chan_4, _chan_5, _chan_6;

    // Safety
    static bool _isArmed;
    static bool _gotFailsafe;

    _board.readData(usec_curr, dt, 
            _chan_1, _chan_2, _chan_3, _chan_4, _chan_5, _chan_6,
            _isArmed, _gotFailsafe,
            thrustDemand, rollDemand, pitchDemand, yawDemand,
            phi, theta, psi, gyroX, gyroY, gyroZ);

    const auto resetPids = thrustDemand < THROTTLE_DOWN;

    // Run demands through PID controllers

    _pitchRollAnglePid.run(
            dt, resetPids, rollDemand, pitchDemand, phi, theta);

    _pitchRollRatePid.run(
            dt, resetPids, rollDemand, pitchDemand, gyroX, gyroY);

    _yawRatePid.run(dt, resetPids, yawDemand, gyroZ);

    float m1_command=0, m2_command=0, m3_command=0, m4_command=0;

    // Run motor mixer
    hf::Mixer::runBetaFlightQuadX(
            thrustDemand, rollDemand, pitchDemand, yawDemand, 
            m1_command, m2_command, m3_command, m4_command);

    // Rescale motor values for OneShot125
    _m1_usec = scaleMotor(m1_command);
    _m2_usec = scaleMotor(m2_command);
    _m3_usec = scaleMotor(m3_command);
    _m4_usec = scaleMotor(m4_command);

    // Turn off motors under various conditions
    cutMotors(_chan_5, _isArmed); 

    // Run motors
    runMotors(); 

    // Get vehicle commands for next loop iteration
    readReceiver(_chan_1, _chan_2, _chan_3, _chan_4, _chan_5, _chan_6,
            _isArmed, _gotFailsafe); 

    _board.runMixer(usec_curr);
}
