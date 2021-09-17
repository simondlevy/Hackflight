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
import Utils(compose)

hackflightPure :: Receiver -> [Sensor] -> [PidController] -> Mixer -> SafetyFun -> (Motors, Safety)

hackflightPure receiver sensors pidControllers mixer safetyFun = (motors, safety)

  where

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
    safety = safetyFun vehicleState

    -- Apply mixer to demands to get motor values, returning motor values and LED state
    motors = mixer safety demands

-------------------------------------------------------------------------------------

hackflightFull :: Receiver -> [Sensor] -> [PidController] -> Mixer -> SafetyFun -> ParserFun ->
    (Motors, Stream Bool, SerialBuffer, Stream Bool)

hackflightFull receiver sensors pidControllers mixer safetyFun parserFun 
  = (motors, led, serialBuffer, starting)

  where

    (motors, safety) = hackflightPure receiver sensors pidControllers mixer safetyFun

    -- Blink LED on startup
    led = if time_msec < 5000 then (mod (div time_msec 50) 2 == 0)
          else armed safety

    -- Run serial comms
    serialBuffer = SerialBuffer 0 0 0 0 0 0 0 0 0 0 0 0 0 0

    -- Helps to init first time around
    starting = [False] ++ true

-------------------------------------------------------------------------------------

hackflightSim :: Receiver -> [Sensor] -> [PidController] -> Mixer -> Motors

hackflightSim receiver sensors pidControllers mixer = motors
   where
     (motors, _) = hackflightPure receiver sensors pidControllers mixer getSafetySim
