{--
  Hackflight core algorithm

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Hackflight where

import Language.Copilot

import Prelude hiding((||), (++), (<), (>), (&&), (==), div, mod, not)

import Safety
import Receiver
import State
import Sensor
import PidController
import Demands
import Mixer
import Time(time_msec)
import Serial
import Utils(compose, isTrue)

hackflight :: Receiver -> [Sensor] -> [PidController] -> Mixer -> Serial ->
    (Motors, Stream Bool, Byte)

hackflight receiver sensors pidControllers mixer serial =
     (motors, led, serialByte)

  where

    -- Allow enough to for startup
    starting = time_msec < 5000

    -- Get receiver demands from external C functions
    receiverDemands = getDemands receiver

    -- Inject the receiver demands into the PID controllers
    pidControllers' = map (\p -> PidController (pidFun p) receiverDemands) pidControllers

    -- Get the vehicle state by running the sensors
    vehicleState = compose sensors zeroState

    -- Map the PID update function to the pid controllers
    pidControllers'' = map (pidUpdate vehicleState) pidControllers'

    -- Sum over the list of demands to get the final demands
    demands = foldr addDemands receiverDemands (map pidDemands pidControllers'')

    -- Map throttle from [-1,+1] to [0,1]
    demands' = Demands (((throttle demands) + 1) / 2)
                       (roll demands)
                       (pitch demands)
                       (yaw demands)

    -- Get safety status
    safety = getSafety starting vehicleState

    -- Apply mixer to demands to get motor values, returning motor values and LED state
    motors = mixer safety demands

    -- Blink LED on startup
    led = (starting && mod (div time_msec 50) 2 == 0) ||
               ((not starting) && (armed safety))

    -- Run serial comms
    serialByte = getSerialByte serial vehicleState receiverDemands 
