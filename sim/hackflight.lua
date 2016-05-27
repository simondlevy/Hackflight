function scalarTo3D(s, a)
    return {s*a[3], s*a[7], s*a[11]}
end

function rotate(x, y, theta)

    return {math.cos(theta)*x + math.sin(theta)*y, -math.sin(theta)*x + math.cos(theta)*y}
end

function round(num, idp)
    if idp and idp>0 then
        local mult = 10^idp
        return math.floor(num * mult + 0.5) / mult
    end
    return math.floor(num + 0.5)
end


threadFunction=function()

    local pluginHandle = simLoadModule('/home/levy/Desktop/simflight/libv_repExtHackflight.so', 'Hackflight')
    simExtHackflight_create(simGetObjectHandle('Quadcopter'))
    simExtHackflight_start()

    while simGetSimulationState()~=sim_simulation_advancing_abouttostop do

        -- Get Euler angles for IMU
        orientation = simGetObjectOrientation(base, -1)

        -- Convert Euler angles to pitch, roll, yaw
        -- See http://en.wikipedia.org/wiki/Flight_dynamics_(fixed-wing_aircraft) for positive/negative orientation
        alpha = orientation[1]
        beta  = orientation[2]
        gamma = orientation[3]

        rollpitch = rotate(alpha, beta, gamma)

        pitchAngle = -rollpitch[2]
        rollAngle  = -rollpitch[1]
        yawAngle   = -gamma

        for i = 1, 4, 1 do

            -- Set forces and torques for each motor
            simExtHackflight_update(i, timestep, pitchAngle, rollAngle, yawAngle)

            thrust = simGetFloatSignal('thrust')

            force = particleCount* PARTICLE_DENSITY * thrust * math.pi * math.pow(PARTICLE_SIZE,3) / (6*timestep)

            torque = math.pow(-1, i+1)*.002 * thrust

            -- Set float signals to the respective motors, and motor respondables
            simSetFloatSignal('Motor'..i..'_respondable', motorRespondableList[i])

            motorMatrix = simGetObjectMatrix(motorList[i],-1)

            forces = scalarTo3D(force,  motorMatrix)
            torques = scalarTo3D(torque, motorMatrix)

            -- Set force and torque for motor
            for k = 1, 3, 1 do
                simSetFloatSignal('force'..i..k,  forces[k])
                simSetFloatSignal('torque'..i..k, torques[k])
            end

        end


        simSwitchThread()
    end

    simExtHackflight_stop()
    simExtHackflight_destroy()
    simUnloadModule(pluginHandle)

end

-- Put some initialization code here:
simSetThreadSwitchTiming(2) -- Default timing for automatic thread switching

local portNb = simGetInt32Parameter(sim_intparam_server_port_next)
local portStart = simGetInt32Parameter(sim_intparam_server_port_start)
local portRange = simGetInt32Parameter(sim_intparam_server_port_range)
local newPortNb = portNb+1
if (newPortNb>=portStart+portRange) then
    newPortNb=portStart
end

simSetInt32Parameter(sim_intparam_server_port_next,newPortNb)
simSetThreadAutomaticSwitch(true)

base = simGetObjectHandle('Quadcopter')

motorList = {}
motorRespondableList = {}

-- Get the object handles for the motors and respondables
for i = 1, 4, 1 do
    motorList[i]=simGetObjectHandle('Motor'..i)
    motorRespondableList[i]=simGetObjectHandle('Motor'..i..'_respondable')
end

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

