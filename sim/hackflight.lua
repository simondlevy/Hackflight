-- Looping -------------------------------------------------------------------------

function scalarTo3D(s, a)
    return {s*a[3], s*a[7], s*a[11]}
end

threadFunction=function()

    -- Launch plugin
    local pluginHandle = simLoadModule('/home/levy/Desktop/hackflight/sim/libv_repExtHackflight.so', 'Hackflight')

    -- Call pluging start() function
    simExtHackflight_start(timestep)

    -- Simulate a gyroscope by first-differencing Euler angles
    eulerPrev = {0, 0, 0}

    -- Loop till user hits stop button
    while simGetSimulationState()~=sim_simulation_advancing_abouttostop do

        -- Get Euler angles for gyroscope simulation
        euler = simGetObjectOrientation(base, -1)
        gyro = {}
        for k = 1,3,1 do
            gyro[k] = (euler[k] - eulerPrev[k]) / timestep
        end
        eulerPrev = euler

        -- Read accelerometer
        result,accel = simReadForceSensor(accelHandle)

        -- Loop over motors
        for i = 1,4,1 do

            -- Send IMU info to plugin
            simExtHackflight_update(i, euler, gyro, accel)

            -- Get motor thrust from plugin
            thrust = simGetFloatSignal('thrust')

            -- Convert thrust to force and torque
            force = particleCount* PARTICLE_DENSITY * thrust * math.pi * math.pow(PARTICLE_SIZE,3) / (6*timestep)
            torque = math.pow(-1, i+1)*.002 * thrust

            -- Set force and torque signals to the respective motors, and motor respondables

            simSetFloatSignal('Motor'..i..'_respondable', motorRespondableList[i])

            motorMatrix = simGetObjectMatrix(motorList[i],-1)

            forces = scalarTo3D(force,  motorMatrix)
            torques = scalarTo3D(torque, motorMatrix)

            for k = 1, 3, 1 do
                simSetFloatSignal('force'..i..k,  forces[k])
                simSetFloatSignal('torque'..i..k, torques[k])
            end

            -- Simulate prop spin based on thrust
            jointAngle = simGetJointPosition(motorJointList[i])
            simSetJointPosition(motorJointList[i], jointAngle + propDirections[i]*thrust/5)

        end -- loop over motors


        simSwitchThread()

    end -- loop till user hits stop button

    -- Cleanup
    simExtHackflight_stop()
    simUnloadModule(pluginHandle)

end

-- Initialization --------------------------------------------------------------------

simSetThreadSwitchTiming(2) -- Default timing for automatic thread switching

simSetThreadAutomaticSwitch(true)

base = simGetObjectHandle('Quadcopter')

motorList = {}
motorRespondableList = {}
motorJointList = {}

-- Get the object handles for the motors, joints, respondables
for i = 1, 4, 1 do
    motorList[i]            = simGetObjectHandle('Motor'..i)
    motorRespondableList[i] = simGetObjectHandle('Motor'..i..'_respondable')
    motorJointList[i]       = simGetObjectHandle('Motor'..i..'_joint')
end

-- Get the handle for the accelerometer
accelHandle = simGetObjectHandle('Accelerometer_forceSensor')

-- Set up directions for prop spin
propDirections = {-1,1,1,-1}

-- Get the particle behavior needed to compute force and torque for each motor
PARTICLE_COUNT_PER_SECOND = 430 --simGetScriptSimulationParameter(sim_handle_self,'PARTICLE_COUNT_PER_SECOND')
PARTICLE_DENSITY = 8500 --simGetScriptSimulationParameter(sim_handle_self,'PARTICLE_DENSITY')
PARTICLE_SIZE = .005  -- particleSizes[i] = baseParticleSize*0.005*simGetObjectSizeFactor(motorList[i]) 

timestep = simGetSimulationTimeStep()

particleCount = math.floor(PARTICLE_COUNT_PER_SECOND * timestep)

-- Here we execute the regular thread code:
res,err=xpcall(threadFunction,function(err) return debug.traceback(err) end)
if not res then
    simAddStatusbarMessage('Lua runtime error: '..err)
end

