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
    _board.readData();

    // Safety
    static bool _isArmed;
    static bool _gotFailsafe;

    static uint32_t chan_1, chan_2, chan_3, chan_4, chan_5, chan_6;

    // Keep track of what time it is and how much time has elapsed since the last loop
    const auto usec_curr = micros();      
    static uint32_t _usec_prev;
    const float dt = (usec_curr - _usec_prev)/1000000.0;
    _usec_prev = usec_curr;      

    // Arm vehicle if safe
    if (!_gotFailsafe && (chan_5 > 1500) && (chan_1 < 1050)) {
        _isArmed = true;
    }

    // LED should be on when armed
    if (_isArmed) {
        digitalWrite(LED_PIN, HIGH);
    }

    // Otherwise, blink LED as heartbeat or failsafe rate
    else {
        _blinkTask.run(LED_PIN, usec_curr,
                _gotFailsafe ? 
                FAILSAFE_BLINK_RATE_HZ : 
                HEARTBEAT_BLINK_RATE_HZ);
    }

    //Get vehicle state

    float AccX = 0, AccY = 0, AccZ = 0;
    float gyroX = 0, gyroY = 0, gyroZ = 0;

    readImu(AccX, AccY, AccZ, gyroX, gyroY, gyroZ); 

    // Get Euler angles from IMU (note negations)
    float phi = 0, theta = 0, psi = 0;
    Madgwick6DOF(dt, gyroX, -gyroY, gyroZ, -AccX, AccY, AccZ, phi, theta, psi);
    psi = -psi;

    // Convert stick demands to appropriate intervals
    float thrustDemand =
        constrain((chan_1 - 1000.0) / 1000.0, 0.0, 1.0);
    float rollDemand = 
        constrain((chan_2 - 1500.0) / 500.0, -1.0, 1.0) * PITCH_ROLL_PRESCALE;
    float pitchDemand =
        constrain((chan_3 - 1500.0) / 500.0, -1.0, 1.0) * PITCH_ROLL_PRESCALE;
    float yawDemand = 
        constrain((chan_4 - 1500.0) / 500.0, -1.0, 1.0) * YAW_PRESCALE;

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
    cutMotors(chan_5, _isArmed); 

    // Run motors
    runMotors(); 

    // Get vehicle commands for next loop iteration
    readReceiver(chan_1, chan_2, chan_3, chan_4, chan_5, chan_6,
            _isArmed, _gotFailsafe); 

    _board.runMixer(usec_curr);
}
