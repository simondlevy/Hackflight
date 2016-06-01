--[[
This script works with the plugin in v_repExtHackflight.cpp. The script passes the
simulated gyroscope and accelerometer values to the plugin's update() method,
which sends the motor thrust values back to the script as signals.

Copyright (C) 2016 Simon D. Levy

This file is part of Hackflight.

Hackflight is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
--]]

-- Looping -------------------------------------------------------------------------

function scalarTo3D(s, a)
    return {s*a[3], s*a[7], s*a[11]}
end

function setColor(handle, signalName, color)
    if (simGetIntegerSignal(signalName)) == 0 then
        color = {0,0,0}
    end
    simSetShapeColor(handle, nil, 0, color)
end

threadFunction=function()

    -- Launch plugin
    local pluginHandle = simLoadModule('/home/levy/Desktop/hackflight/sim/libv_repExtHackflight.so', 'Hackflight')

    -- Call pluging start() function
    simExtHackflight_start(timestep)

    -- Simulate a gyroscope by first-differencing pitch,roll,yaw angles
    anglesPrev = {0, 0, 0}

    -- Loop till user hits stop button
    while simGetSimulationState()~=sim_simulation_advancing_abouttostop do

        -- Get Euler angles for gyroscope simulation
        euler = simGetObjectOrientation(quadcopterHandle, -1)

        -- Convert Euler angles to pitch and roll via rotation formula
        angles = {0,0,0}
        angles[1] =  math.sin(euler[3])*euler[1] - math.cos(euler[3])*euler[2];
        angles[2] = -math.cos(euler[3])*euler[1] - math.sin(euler[3])*euler[2]; 
        angles[3] = -euler[3] -- yaw direct from Euler

        -- Compute pitch, roll, yaw first derivative to simulate gyro
        gyro = {0,0,0}
        for k = 1,3,1 do
            gyro[k] = (angles[k] - anglesPrev[k]) / timestep
        end
        anglesPrev = angles

        -- Read accelerometer
        result,accel = simReadForceSensor(accelHandle)

        -- Send IMU info to plugin
        simExtHackflight_update(gyro, accel)

        -- Loop over motors
        for i = 1,4,1 do

            -- Get motor thrust from plugin
            thrust = simGetFloatSignal('thrust'..i)

            -- Convert thrust to force and torque
            force = particleCount* PARTICLE_DENSITY * thrust * math.pi * math.pow(PARTICLE_SIZE,3) / (6*timestep)
            torque = math.pow(-1, i+1)*.002 * thrust

            -- Compute force and torque signals based on thrust
            simSetFloatSignal('Motor'..i..'_respondable', motorRespondableList[i])
            motorMatrix = simGetObjectMatrix(motorList[i],-1)
            forces = scalarTo3D(force,  motorMatrix)
            torques = scalarTo3D(torque, motorMatrix)

            -- Send force and torque to props
            for k = 1, 3, 1 do
                simSetFloatSignal('force'..i..k,  forces[k])
                simSetFloatSignal('torque'..i..k, torques[k])
            end

            -- Simulate prop spin based on thrust
            jointAngle = simGetJointPosition(motorJointList[i])
            print(thrust)
            simSetJointPosition(motorJointList[i], jointAngle + propDirections[i]*thrust/5)

        end -- loop over motors

        -- Set LEDs based on signals from plugin
        setColor(greenHandle, 'greenLED', {0,1,0})
        setColor(redHandle,   'redLED',   {1,0,0})

        simSwitchThread()

    end -- loop till user hits stop button

    -- Cleanup
    simExtHackflight_stop()
    simUnloadModule(pluginHandle)

end

-- Initialization --------------------------------------------------------------------

-- Set default timing for automatic thread switching
simSetThreadSwitchTiming(2) 
simSetThreadAutomaticSwitch(true)

-- Get the object handles for the motors, joints, respondables
motorList = {}
motorRespondableList = {}
motorJointList = {}
for i = 1, 4, 1 do
    motorList[i]            = simGetObjectHandle('Motor'..i)
    motorRespondableList[i] = simGetObjectHandle('Motor'..i..'_respondable')
    motorJointList[i]       = simGetObjectHandle('Motor'..i..'_joint')
end

-- Get handle for objects we'll access
quadcopterHandle = simGetObjectHandle('Quadcopter')
accelHandle = simGetObjectHandle('Accelerometer_forceSensor')
greenHandle = simGetObjectHandle('Green_LED_visible')
redHandle   = simGetObjectHandle('Red_LED_visible')

-- Set up directions for prop spin
propDirections = {-1,1,1,-1}

-- Timestep is used in various places
timestep = simGetSimulationTimeStep()

-- Set the particle behavior needed to compute force and torque for each motor
PARTICLE_COUNT_PER_SECOND = 430 
PARTICLE_DENSITY          = 8500 
PARTICLE_SIZE             = .005
particleCount = math.floor(PARTICLE_COUNT_PER_SECOND * timestep)

-- Here we execute the regular thread code:
res,err=xpcall(threadFunction,function(err) return debug.traceback(err) end)
if not res then
    simAddStatusbarMessage('Lua runtime error: '..err)
end

